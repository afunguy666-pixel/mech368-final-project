// SIMULATOR DEMO ONLY - not robot firmware and not part of the course build.
// Shows how ordinary Arduino code for motors, encoders, line sensors, an
// ultrasonic sensor, APDS gestures, the IMU and a battery divider runs in the
// firmware simulator. Pins match sim/vehicles/example.vehicle (placeholders).
// It does NOT use the BLE link or the MotionSafety gate; real autonomous modes
// must share the robot's single safety-gated output path.
//
// Behaviour: wait for an UP gesture, follow the tape line with encoder speed
// control, stop for obstacles closer than 20 cm, stop on a DOWN gesture, and
// light the red LED when the battery is low.
// Mapping: send "pose X Y HEADING_DEG" on Serial to give the starting pose.
// The sketch then tracks its pose (encoder distance + gyro heading, gyro bias
// calibrated while waiting) and prints "MAP x y tempC" every 150 ms.
#include <Arduino.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_HS300x.h>

const int LEFT_PWM = D5, LEFT_DIR = D4, RIGHT_PWM = D6, RIGHT_DIR = D7;
const int LEFT_ENC_A = D2, LEFT_ENC_B = D3, RIGHT_ENC_A = D8, RIGHT_ENC_B = D9;
const int LINE_LEFT = A0, LINE_CENTRE = A1, LINE_RIGHT = A2;
const int TRIG = D10, ECHO = D11, BATTERY = A7;

const float WHEEL_RADIUS = 0.020f, TRACK = 0.10f;
const float COUNTS_PER_REV = 720.0f;   // 360 ppr, both edges of channel A
const float MAX_WHEEL_SPEED = 30.0f;   // rad/s, for the feed-forward term
const float CRUISE = 0.25f;            // m/s
const float STOP_DISTANCE = 0.20f, CLEAR_DISTANCE = 0.30f;
const float LOW_BATTERY_VOLTS = 6.8f;

volatile long leftCount = 0, rightCount = 0;
void leftEncoderIsr() { leftCount += digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B) ? -1 : 1; }
void rightEncoderIsr() { rightCount += digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B) ? -1 : 1; }

enum State { WAITING, FOLLOWING, BLOCKED, STOPPED };
State state = WAITING;
const char* stateName(State s) {
  switch (s) {
    case WAITING: return "waiting";
    case FOLLOWING: return "following";
    case BLOCKED: return "blocked";
    case STOPPED: return "stopped";
  }
  return "?";
}

float targetLeft = 0, targetRight = 0;  // wheel rad/s
float integralLeft = 0, integralRight = 0;
long lastLeftCount = 0, lastRightCount = 0;
float linePosition = 0, lastLinePosition = 0;
float obstacle = -1;                     // metres, -1 = nothing in range
float odoX = 0, odoY = 0, odoHeading = 0, gyroHeading = 0;
bool lowBattery = false;
bool mapping = false;                    // set by the "pose" serial command
float poseX = 0, poseY = 0, poseHeading = 0;  // encoder distance + gyro heading
float gyroBias = 0, biasSum = 0;
long biasCount = 0;
unsigned long lastMap = 0, lastGyroUs = 0;
unsigned long lastControl = 0, lastLine = 0, lastSonar = 0, lastBattery = 0, lastReport = 0;

void drive(int pwmPin, int dirPin, float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  digitalWrite(dirPin, duty >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, static_cast<int>(fabsf(duty) * 255.0f + 0.5f));
}

void setState(State next) {
  if (next == state) return;
  Serial.print("state: ");
  Serial.print(stateName(state));
  Serial.print(" -> ");
  Serial.println(stateName(next));
  state = next;
  digitalWrite(LEDG, state == FOLLOWING ? LOW : HIGH);  // RGB LEDs are active low
  digitalWrite(LEDB, state == WAITING ? LOW : HIGH);
}

// Weighted line position: +1 = line under the left sensor, -1 = right.
void readLine() {
  const float left = analogRead(LINE_LEFT) / 1023.0f;
  const float centre = analogRead(LINE_CENTRE) / 1023.0f;
  const float right = analogRead(LINE_RIGHT) / 1023.0f;
  const float floorLevel = 0.15f;  // ~0.4 V over the floor
  const float l = max(0.0f, left - floorLevel), c = max(0.0f, centre - floorLevel),
              r = max(0.0f, right - floorLevel);
  const float sum = l + c + r;
  lastLinePosition = linePosition;
  if (sum > 0.05f) linePosition = (l - r) / sum;
  else linePosition = linePosition >= 0 ? 1.5f : -1.5f;  // lost: keep turning toward it
}

void readSonar() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  const unsigned long echo = pulseIn(ECHO, HIGH, 12000UL);  // ~2 m max
  obstacle = echo ? echo * 343.0f / 2.0f / 1e6f : -1.0f;
}

void speedControl(float dt) {
  noInterrupts();
  const long l = leftCount, r = rightCount;
  interrupts();
  const float wl = (l - lastLeftCount) / COUNTS_PER_REV * TWO_PI / dt;
  const float wr = (r - lastRightCount) / COUNTS_PER_REV * TWO_PI / dt;
  const float dl = (l - lastLeftCount) / COUNTS_PER_REV * TWO_PI * WHEEL_RADIUS;
  const float dr = (r - lastRightCount) / COUNTS_PER_REV * TWO_PI * WHEEL_RADIUS;
  lastLeftCount = l;
  lastRightCount = r;
  const float ds = (dl + dr) / 2, dyaw = (dr - dl) / TRACK;
  odoX += ds * cosf(odoHeading + dyaw / 2);
  odoY += ds * sinf(odoHeading + dyaw / 2);
  odoHeading += dyaw;
  poseX += ds * cosf(poseHeading);
  poseY += ds * sinf(poseHeading);

  const float kp = 0.02f, ki = 0.15f;
  integralLeft = constrain(integralLeft + (targetLeft - wl) * dt, -2.0f, 2.0f);
  integralRight = constrain(integralRight + (targetRight - wr) * dt, -2.0f, 2.0f);
  if (targetLeft == 0 && targetRight == 0) {
    integralLeft = integralRight = 0;
    drive(LEFT_PWM, LEFT_DIR, 0);
    drive(RIGHT_PWM, RIGHT_DIR, 0);
    return;
  }
  drive(LEFT_PWM, LEFT_DIR, targetLeft / MAX_WHEEL_SPEED + kp * (targetLeft - wl) + ki * integralLeft);
  drive(RIGHT_PWM, RIGHT_DIR, targetRight / MAX_WHEEL_SPEED + kp * (targetRight - wr) + ki * integralRight);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);
  drive(LEFT_PWM, LEFT_DIR, 0);
  drive(RIGHT_PWM, RIGHT_DIR, 0);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderIsr, CHANGE);
  if (!IMU.begin()) Serial.println("IMU init failed");
  if (!APDS.begin()) Serial.println("APDS init failed");
  if (!HS300x.begin()) Serial.println("HS300x init failed");
  Serial.println("line follower ready: swipe UP to start, DOWN to stop");
}

void loop() {
  const unsigned long now = millis();

  if (APDS.gestureAvailable()) {
    const int g = APDS.readGesture();
    if (g == GESTURE_UP && (state == WAITING || state == STOPPED)) {
      Serial.println("gesture UP: started");
      setState(FOLLOWING);
    } else if (g == GESTURE_DOWN) {
      Serial.println("gesture DOWN: stopping");
      setState(STOPPED);
    }
  }

  // "pose X Y HEADING_DEG": the known starting pose; enables mapping output.
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("pose ")) {
      line = line.substring(5);
      const int a = line.indexOf(' '), b = line.indexOf(' ', a + 1);
      if (a > 0 && b > a) {
        poseX = line.substring(0, a).toFloat();
        poseY = line.substring(a + 1, b).toFloat();
        poseHeading = radians(line.substring(b + 1).toFloat());
        mapping = true;
        Serial.println("mapping enabled");
      }
    }
  }

  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    // Integrate over the real time since the last read, not a fixed 10 ms:
    // blocking calls (temperature read, pulseIn) make the loop skip samples.
    const unsigned long nowUs = micros();
    const float dt = lastGyroUs ? (nowUs - lastGyroUs) / 1e6f : 0.01f;
    lastGyroUs = nowUs;
    if (state == WAITING) {  // standing still: estimate the gyro bias
      biasSum += gz;
      gyroBias = biasSum / ++biasCount;
    } else {
      gyroHeading += (gz - gyroBias) * dt;  // degrees
      poseHeading += radians((gz - gyroBias) * dt);
    }
  }

  if (now - lastSonar >= 60) {
    lastSonar = now;
    readSonar();
    if (state == FOLLOWING && obstacle > 0 && obstacle < STOP_DISTANCE) {
      Serial.print("obstacle at ");
      Serial.print(obstacle * 100, 1);
      Serial.println(" cm: stopping");
      setState(BLOCKED);
    } else if (state == BLOCKED && (obstacle < 0 || obstacle > CLEAR_DISTANCE)) {
      Serial.println("path clear: resuming");
      setState(FOLLOWING);
    }
  }

  if (now - lastLine >= 5) {
    lastLine = now;
    readLine();
    if (state == FOLLOWING) {
      const float error = linePosition;
      const float omega = 6.0f * error + 0.25f * (linePosition - lastLinePosition) / 0.005f;
      const float v = CRUISE * (1.0f - 0.5f * min(1.0f, fabsf(error)));
      targetLeft = (v - omega * TRACK / 2) / WHEEL_RADIUS;
      targetRight = (v + omega * TRACK / 2) / WHEEL_RADIUS;
    } else {
      targetLeft = targetRight = 0;
    }
  }

  if (now - lastControl >= 20) {
    speedControl((now - lastControl) / 1000.0f);
    lastControl = now;
  }

  if (mapping && state == FOLLOWING && now - lastMap >= 150) {
    lastMap = now;
    const float celsius = HS300x.readTemperature();
    Serial.print("MAP ");
    Serial.print(poseX, 3);
    Serial.print(' ');
    Serial.print(poseY, 3);
    Serial.print(' ');
    Serial.println(celsius, 2);
  }

  if (now - lastBattery >= 500) {
    lastBattery = now;
    const float volts = analogRead(BATTERY) * 3.3f / 1023.0f * 3.0f;
    if (!lowBattery && volts < LOW_BATTERY_VOLTS) {
      lowBattery = true;
      Serial.print("LOW BATTERY: ");
      Serial.print(volts, 2);
      Serial.println(" V");
      digitalWrite(LEDR, LOW);
    }
  }

  if (now - lastReport >= 1000) {
    lastReport = now;
    Serial.print(stateName(state));
    Serial.print(" odo=(");
    Serial.print(odoX, 3);
    Serial.print(", ");
    Serial.print(odoY, 3);
    Serial.print(") m heading enc=");
    Serial.print(degrees(odoHeading), 1);
    Serial.print(" gyro=");
    Serial.print(gyroHeading, 1);
    Serial.print(" deg line=");
    Serial.print(linePosition, 2);
    Serial.print(" sonar=");
    Serial.println(obstacle, 2);
  }
}
