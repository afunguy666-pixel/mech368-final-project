// Line tracker: autonomous track following with IR pairs, built on
// libraries/Mech368Line. Runs on the Nano 33 BLE Sense Rev2 and in the
// firmware simulator (sim/vehicles/ir_tracker.vehicle and
// ir_tracker_lm393.vehicle; pins and geometry here must match the vehicle).
//
// NOT the course firmware yet: it has no BLE link and no MotionSafety gate.
// Before it drives a real robot, merge it into firmware/robot so every motor
// command passes the robot's single safety-gated output (see
// motionPermitted() below) and the BLE E-stop stops the navigator.
//
// Hardware (course kit):
//   - Line array: 4 x LTE-4206 / LTR-4206, 3 cm apart, 6 cm ahead of the
//     wheel axle, on A0..A3 (index 0 = rightmost). Either raw phototransistor
//     voltages (analog, best) or LM393 outputs (digital) - the same code
//     reads both, the calibration sweep finds each sensor's range.
//   - Obstacle IR pairs through an LM393 at the front corners: D12 (left),
//     D11 (right), output LOW when an object is within range.
//   - DRV8833 (IN/IN): AIN1/AIN2 = D4/D5 (left), BIN1/BIN2 = D6/D7 (right),
//     nSLEEP = D8 (LOW = outputs off), nFAULT -> A6 (open drain, pulled up).
//   - Encoders on D2/D3 (left) and D9/D10 (right), channel A on interrupts.
//   - All sensor outputs must be pulled up to 3.3 V, never to 5 V.
//
// Operation: place the robot on the start of the track. While it stands
// still the gyro bias is measured. Swipe UP over the APDS-9960: the robot
// sweeps +-40 degrees to calibrate the line sensors, then follows the track,
// takes the 90-degree corners, goes around cones and stops at the end.
// Swipe DOWN to stop.
#include <Arduino.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <Mech368Line.h>

using namespace mech368::line;

// ---------------------------------------------------------------- wiring ---
const int LEFT_IN1 = D4, LEFT_IN2 = D5;     // DRV8833 AIN1, AIN2
const int RIGHT_IN1 = D6, RIGHT_IN2 = D7;   // DRV8833 BIN1, BIN2
const int DRIVER_SLEEP = D8;                // DRV8833 nSLEEP: HIGH = enabled
const int DRIVER_FAULT = A6;                // DRV8833 nFAULT: LOW = over-current/over-temperature
const int LEFT_ENC_A = D2, LEFT_ENC_B = D3, RIGHT_ENC_A = D9, RIGHT_ENC_B = D10;
const int OBSTACLE_LEFT = D12, OBSTACLE_RIGHT = D11;
const int OBSTACLE_ACTIVE = LOW;  // LM393 output level with an object in range

const int LINE_COUNT = 4;
const int LINE_PINS[LINE_COUNT] = {A0, A1, A2, A3};
const float LINE_Y[LINE_COUNT] = {-0.045f, -0.015f, 0.015f, 0.045f};  // m, + = left
const bool BLACK_IS_HIGH = true;     // pull-up wiring: less reflection -> higher voltage
const uint16_t MIN_CAL_SPAN = 200;   // ADC counts (10 bit) between floor and tape

// --------------------------------------------------------------- geometry ---
const float TRACK = 0.11f;            // m between wheel centres
const float WHEEL_RADIUS = 0.016f;    // m
const float COUNTS_PER_REV = 600.0f;  // 300 ppr on channel A, both edges
const float MAX_WHEEL_SPEED = 0.45f;  // m/s, a bit under the motor's top speed
const float CAL_SWEEP = 40.0f;        // degrees each side

volatile long leftCount = 0, rightCount = 0;
void leftEncoderIsr() { leftCount += digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B) ? -1 : 1; }
void rightEncoderIsr() { rightCount += digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B) ? -1 : 1; }

LineSensorArray sensors;
LineNavigator nav;
Odometry odometry;
GyroBias gyroBias;
WheelSpeedPI leftPI, rightPI;

enum Mode { WAITING, CALIBRATING, RUNNING, STOPPED };
Mode mode = WAITING;
bool calibrated = false;
int calPhase = 0;
float calStart = 0;

float heading = 0, headingAtLastTick = 0;  // rad, gyro-integrated
unsigned long lastGyroUs = 0;
long lastLeftTick = 0, lastRightTick = 0, lastLeftSpeed = 0, lastRightSpeed = 0;
float targetLeft = 0, targetRight = 0, speedLeft = 0, speedRight = 0;  // m/s
uint32_t lastTransitions = 0;
bool faultReported = false;
unsigned long lastTick = 0, lastSpeedLoop = 0, lastReport = 0;
LineReading reading;

// Hook for the robot's safety gate (BLE watchdog + E-stop latch). In the
// course firmware this must return MotionSafety::motionAllowed(now).
bool motionPermitted() { return true; }

float metresPerCount() { return TWO_PI * WHEEL_RADIUS / COUNTS_PER_REV; }

// DRV8833 in slow decay (drive/brake): forward = IN1 high, PWM on IN2 with
// the inverted duty; reverse = the mirror image; zero = both high = brake.
// Slow decay gives speed roughly proportional to duty, which helps at the
// low speeds of corners and detours. Both pins always use analogWrite (255 =
// high, 0 = low) so a pin is never switched between PWM and digitalWrite.
void drive(int in1, int in2, float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  const int off = static_cast<int>((1.0f - fabsf(duty)) * 255.0f + 0.5f);
  analogWrite(in1, duty >= 0 ? 255 : off);
  analogWrite(in2, duty >= 0 ? off : 255);
}
void stopMotors() {
  targetLeft = targetRight = 0;
  leftPI.reset();
  rightPI.reset();
  drive(LEFT_IN1, LEFT_IN2, 0);  // brake
  drive(RIGHT_IN1, RIGHT_IN2, 0);
}

void readLineRaw(uint16_t* raw) {
  for (int i = 0; i < LINE_COUNT; ++i) raw[i] = static_cast<uint16_t>(analogRead(LINE_PINS[i]));
}

void printPose(const char* prefix) {
  Serial.print(prefix);
  Serial.print(" at (");
  Serial.print(odometry.pose().x, 3);
  Serial.print(", ");
  Serial.print(odometry.pose().y, 3);
  Serial.print(") m, heading ");
  Serial.print(degrees(heading), 1);
  Serial.println(" deg");
}

void startNavigation() {
  NavInputs in;
  in.line = reading;
  in.heading = heading;
  in.distance = odometry.distance();
  nav.start(in);
  lastTransitions = nav.transitions();
  mode = RUNNING;
  Serial.print("nav: ");
  Serial.print(navStateName(nav.state()));
  Serial.print(" (");
  Serial.print(nav.reason());
  Serial.println(")");
}

void finishCalibration() {
  Serial.print("calibration min/max:");
  for (int i = 0; i < LINE_COUNT; ++i) {
    Serial.print(' ');
    Serial.print(sensors.calibrationMin(i));
    Serial.print('/');
    Serial.print(sensors.calibrationMax(i));
  }
  Serial.println();
  if (!sensors.calibrationValid(MIN_CAL_SPAN)) {
    Serial.println("calibration FAILED: a sensor never saw both tape and floor");
    mode = STOPPED;
    stopMotors();
    return;
  }
  calibrated = true;
  uint16_t raw[LINE_COUNT];
  readLineRaw(raw);
  reading = sensors.readRaw(raw);
  // Placed on the line, the inner sensors should be darker than the outer.
  if (reading.value[1] + reading.value[2] < reading.value[0] + reading.value[3])
    Serial.println("warning: inner sensors read lighter than outer - check BLACK_IS_HIGH");
  startNavigation();
}

// Calibration sweep: rotate to +CAL_SWEEP, -CAL_SWEEP, back to the start.
Twist calibrationStep(const uint16_t* raw) {
  sensors.calibrate(raw);
  const float targets[3] = {radians(CAL_SWEEP), -radians(CAL_SWEEP), 0.0f};
  const float err = wrapAngle(calStart + targets[calPhase] - heading);
  if (fabsf(err) < radians(2.0f)) {
    if (++calPhase == 3) {
      finishCalibration();
      return Twist();
    }
  }
  float w = constrain(4.0f * err, -2.0f, 2.0f);
  if (fabsf(w) < 0.8f) w = w < 0 ? -0.8f : 0.8f;
  return Twist(0.0f, w);
}

void setup() {
  Serial.begin(115200);
  pinMode(DRIVER_SLEEP, OUTPUT);
  digitalWrite(DRIVER_SLEEP, LOW);  // driver off until something is allowed to move
  for (int pin : {LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2}) pinMode(pin, OUTPUT);
  stopMotors();
  pinMode(DRIVER_FAULT, INPUT_PULLUP);
  for (int pin : {LEFT_ENC_A, LEFT_ENC_B, RIGHT_ENC_A, RIGHT_ENC_B, OBSTACLE_LEFT, OBSTACLE_RIGHT})
    pinMode(pin, INPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH);  // RGB LEDs are active low
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);   // blue: waiting
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderIsr, CHANGE);
  if (!IMU.begin()) Serial.println("IMU init failed");
  if (!APDS.begin()) Serial.println("APDS init failed");

  sensors.configure(LINE_COUNT, LINE_Y);
  sensors.blackIsHigh = BLACK_IS_HIGH;
  odometry.reset(Pose());
  Serial.println("line tracker ready: keep still, then swipe UP to calibrate and start, DOWN to stop");
}

void loop() {
  const unsigned long now = millis();

  if (APDS.gestureAvailable()) {
    const int g = APDS.readGesture();
    if (g == GESTURE_UP && (mode == WAITING || mode == STOPPED)) {
      if (!gyroBias.ready(50)) {
        Serial.println("gesture UP ignored: gyro bias not measured yet, keep still");
      } else if (!calibrated) {
        Serial.println("gesture UP: calibrating line sensors");
        sensors.resetCalibration();
        calPhase = 0;
        calStart = heading;
        mode = CALIBRATING;
      } else {
        Serial.println("gesture UP: starting");
        startNavigation();
      }
    } else if (g == GESTURE_DOWN && mode != WAITING) {
      Serial.println("gesture DOWN: stopping");
      nav.stop();
      mode = STOPPED;
      stopMotors();
    }
  }

  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    const unsigned long nowUs = micros();
    const float dt = lastGyroUs ? (nowUs - lastGyroUs) / 1e6f : 0.01f;
    lastGyroUs = nowUs;
    if (mode == WAITING) gyroBias.add(gz);  // standing still
    else heading = wrapAngle(heading + radians((gz - gyroBias.bias()) * dt));
  }

  if (now - lastTick >= 5) {
    const float dt = lastTick ? (now - lastTick) / 1000.0f : 0.005f;
    lastTick = now;
    noInterrupts();
    const long l = leftCount, r = rightCount;
    interrupts();
    const float dl = (l - lastLeftTick) * metresPerCount(), dr = (r - lastRightTick) * metresPerCount();
    lastLeftTick = l;
    lastRightTick = r;
    odometry.updateWithGyro(dl, dr, wrapAngle(heading - headingAtLastTick));
    headingAtLastTick = heading;

    uint16_t raw[LINE_COUNT];
    readLineRaw(raw);
    Twist cmd;
    if (mode == CALIBRATING) {
      cmd = calibrationStep(raw);
    } else if (mode == RUNNING) {
      reading = sensors.readRaw(raw);
      NavInputs in;
      in.dt = dt;
      in.line = reading;
      in.obstacleLeft = digitalRead(OBSTACLE_LEFT) == OBSTACLE_ACTIVE;
      in.obstacleRight = digitalRead(OBSTACLE_RIGHT) == OBSTACLE_ACTIVE;
      in.heading = heading;
      in.distance = odometry.distance();
      cmd = nav.update(in);
      if (nav.transitions() != lastTransitions) {
        lastTransitions = nav.transitions();
        Serial.print("nav: ");
        Serial.print(navStateName(nav.state()));
        Serial.print(" (");
        Serial.print(nav.reason());
        printPose(")");
      }
      if (!nav.running()) {
        mode = STOPPED;
        Serial.print("nav stopped: ");
        Serial.print(navStateName(nav.state()));
        Serial.print(", ");
        Serial.print(nav.corners());
        Serial.print(" corners, ");
        Serial.print(nav.detours());
        Serial.println(" detours");
      }
    }
    const bool moving = motionPermitted() && (mode == CALIBRATING || mode == RUNNING);
    if (!moving) cmd = Twist();
    // Second, independent stop: nSLEEP puts the DRV8833 outputs in high impedance.
    digitalWrite(DRIVER_SLEEP, moving ? HIGH : LOW);
    if (moving && digitalRead(DRIVER_FAULT) == LOW && !faultReported) {
      faultReported = true;
      Serial.println("DRV8833 FAULT: over-current or over-temperature (stalled motor?)");
    }
    const WheelSpeeds w = toWheels(limitTwist(cmd, TRACK, MAX_WHEEL_SPEED), TRACK);
    targetLeft = w.left;
    targetRight = w.right;
    digitalWrite(LEDG, mode == RUNNING ? LOW : HIGH);
    digitalWrite(LEDB, mode == WAITING || mode == CALIBRATING ? LOW : HIGH);
    digitalWrite(LEDR, mode == STOPPED && nav.state() != NavState::Finished ? LOW : HIGH);
  }

  if (now - lastSpeedLoop >= 10) {
    const float dt = lastSpeedLoop ? (now - lastSpeedLoop) / 1000.0f : 0.01f;
    lastSpeedLoop = now;
    noInterrupts();
    const long l = leftCount, r = rightCount;
    interrupts();
    speedLeft = (l - lastLeftSpeed) * metresPerCount() / dt;
    speedRight = (r - lastRightSpeed) * metresPerCount() / dt;
    lastLeftSpeed = l;
    lastRightSpeed = r;
    if (targetLeft == 0 && targetRight == 0) {
      stopMotors();
    } else {
      drive(LEFT_IN1, LEFT_IN2, leftPI.update(targetLeft, speedLeft, dt));
      drive(RIGHT_IN1, RIGHT_IN2, rightPI.update(targetRight, speedRight, dt));
    }
  }

  if (mode == RUNNING && now - lastReport >= 1000) {
    lastReport = now;
    Serial.print(navStateName(nav.state()));
    Serial.print(" line=");
    Serial.print(reading.onLine ? reading.position * 100.0f : NAN, 1);
    Serial.print(" cm");
    printPose("");
  }
}
