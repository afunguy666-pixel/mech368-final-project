#include "DirectLineTracker.h"
#include <Arduino.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>

namespace mech368 {
using namespace line;

namespace {
constexpr int kLeftIn1 = D4, kLeftIn2 = D5, kRightIn1 = D6, kRightIn2 = D7;
constexpr int kSleep = D8, kFault = A6;
constexpr int kLeftA = D2, kLeftB = D3, kRightA = D9, kRightB = D10;
constexpr int kObstacleLeft = D12, kObstacleRight = D11;  // LM393 active low
constexpr int kLinePins[] = {A0, A1, A2, A3};            // right to left
constexpr float kLineY[] = {-0.030f, -0.010f, 0.010f, 0.030f};
constexpr bool BLACK_IS_HIGH = true;
constexpr int OBSTACLE_ACTIVE = LOW;
constexpr uint16_t MIN_CAL_SPAN = 200;
constexpr float kTrack = 0.092f, kRadius = 0.0325f;
// The 14.9 mm caster spacer levels the chassis. A probe normal to the
// clamp samples the floor directly below its centreline.
constexpr float kSensorAhead = 0.07575f;
constexpr float kTapeWidth = 0.050f, kMaxWheelSpeed = 0.45f;
// Bench measurements pending: 300 channel-A pulses/rev, counted on both edges.
constexpr float kCountsPerRev = 600.0f, kMotorTopSpeed = 0.95f;

float wheelTravel(uint32_t count, uint32_t previous) {
  return static_cast<int32_t>(count - previous) * (2.0f * kPi * kRadius / kCountsPerRev);
}

void readLine(uint16_t* raw) {
  for (int i = 0; i < 4; ++i) raw[i] = analogRead(kLinePins[i]);
}

// DRV8833 slow decay: one input stays high, the other carries inverted PWM.
// At zero both inputs are high (brake). nSLEEP low disables both bridges.
void drive(int in1, int in2, float duty) {
  duty = clampf(duty, -1.0f, 1.0f);
  const int off = static_cast<int>((1.0f - fabsf(duty)) * 255.0f + 0.5f);
  analogWrite(in1, duty >= 0 ? 255 : off);
  analogWrite(in2, duty >= 0 ? off : 255);
}
}  // namespace

void DirectLineTracker::begin(void (*leftEncoderIsr)(), void (*rightEncoderIsr)()) {
  pinMode(kSleep, OUTPUT);
  digitalWrite(kSleep, LOW);
  for (int pin : {kLeftIn1, kLeftIn2, kRightIn1, kRightIn2}) pinMode(pin, OUTPUT);
  disableDrive();
  pinMode(kFault, INPUT_PULLUP);
  for (int pin : {kLeftA, kLeftB, kRightA, kRightB, kObstacleLeft, kObstacleRight})
    pinMode(pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(kLeftA), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kRightA), rightEncoderIsr, CHANGE);
  for (int pin : {LEDR, LEDG, LEDB}) pinMode(pin, OUTPUT);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);
  Serial.begin(115200);
  imuReady_ = IMU.begin();
  gestureReady_ = APDS.begin();
  if (!imuReady_) Serial.println("IMU init failed");
  if (!gestureReady_) Serial.println("APDS init failed");
  sensors_.configure(4, kLineY);
  sensors_.blackIsHigh = BLACK_IS_HIGH;
  nav_.cfg.sensorAhead = kSensorAhead;
  nav_.cfg.cornerAdvance = nav_.cfg.rejoinAdvance = kSensorAhead + kTapeWidth / 2;
  // Three dark sensors also occur on a slightly off-centre straight.
  nav_.cfg.confirmCornerByLoss = true;
  nav_.cfg.cornerAdvanceAfterLoss = kSensorAhead - kTapeWidth / 2;
  nav_.cfg.sideTolerance = 0.018f;
  nav_.cfg.centredTolerance = 0.008f;
  leftPI_.feedForward = rightPI_.feedForward = 1.0f / kMotorTopSpeed;
}

void DirectLineTracker::leftEdge() { leftCount_ += digitalRead(kLeftA) == digitalRead(kLeftB) ? -1 : 1; }
void DirectLineTracker::rightEdge() { rightCount_ += digitalRead(kRightA) == digitalRead(kRightB) ? -1 : 1; }

int DirectLineTracker::readGesture() {
  return gestureReady_ && APDS.gestureAvailable() ? APDS.readGesture() : GESTURE_NONE;
}

void DirectLineTracker::disableDrive() {
  digitalWrite(kSleep, LOW);
  enabled_ = false;
  targets_ = WheelSpeeds();
  leftPI_.reset();
  rightPI_.reset();
  drive(kLeftIn1, kLeftIn2, 0);
  drive(kRightIn1, kRightIn2, 0);
}

void DirectLineTracker::stop() {
  if (!active()) return;
  disableDrive();
  nav_.stop();
  mode_ = Mode::Stopped;
}

void DirectLineTracker::start() {
  if (active() || !motionPermitted_()) return;
  if (digitalRead(kFault) == LOW) {
    Serial.println("DRV8833 FAULT: start inhibited");
    return;
  }
  if (!gyroBias_.ready(50)) {
    Serial.println("gesture UP ignored: gyro bias not measured yet, keep still");
    return;
  }
  if (calibrated_) {
    startNavigation();
  } else {
    sensors_.resetCalibration();
    calPhase_ = 0;
    // A stop partway through a sweep leaves the chassis turned. Repeat the
    // entire sweep about the original track heading after an explicit restart.
    if (!haveCalStart_) {
      calStart_ = heading_;
      haveCalStart_ = true;
    }
    mode_ = Mode::Calibrating;
    Serial.println("gesture UP: calibrating line sensors");
  }
}

void DirectLineTracker::startNavigation() {
  uint16_t raw[4];
  readLine(raw);
  reading_ = sensors_.readRaw(raw);
  NavInputs in;
  in.line = reading_;
  in.heading = heading_;
  in.distance = odometry_.distance();
  nav_.start(in);
  mode_ = Mode::Running;
  Serial.print("nav: ");
  Serial.print(navStateName(nav_.state()));
  Serial.print(" (");
  Serial.print(nav_.reason());
  Serial.println(")");
}

Twist DirectLineTracker::calibrate(const uint16_t* raw) {
  sensors_.calibrate(raw);
  const float angles[] = {degToRad(40), degToRad(-40), 0};
  const float error = wrapAngle(calStart_ + angles[calPhase_] - heading_);
  if (fabsf(error) < degToRad(2) && ++calPhase_ == 3) {
    Serial.print("calibration min/max:");
    for (int i = 0; i < 4; ++i) {
      Serial.print(' ');
      Serial.print(sensors_.calibrationMin(i));
      Serial.print('/');
      Serial.print(sensors_.calibrationMax(i));
    }
    Serial.println();
    haveCalStart_ = false;
    calibrated_ = sensors_.calibrationValid(MIN_CAL_SPAN);
    if (calibrated_) {
      startNavigation();
      if (reading_.value[1] + reading_.value[2] < reading_.value[0] + reading_.value[3])
        Serial.println("warning: inner sensors read lighter than outer - check sensor polarity");
    } else {
      stop();
      Serial.println("calibration FAILED: a sensor never saw both tape and floor");
    }
    return Twist();
  }
  float omega = clampf(4.0f * error, -2.0f, 2.0f);
  if (fabsf(omega) < 0.8f) omega = omega < 0 ? -0.8f : 0.8f;
  return Twist(0, omega);
}

void DirectLineTracker::printPose() {
  Serial.print(" at (");
  Serial.print(odometry_.pose().x, 3);
  Serial.print(", ");
  Serial.print(odometry_.pose().y, 3);
  Serial.print(") m, heading ");
  Serial.print(radToDeg(heading_), 1);
  Serial.println(" deg");
}

Twist DirectLineTracker::navigate(const uint16_t* raw, float dt) {
  reading_ = sensors_.readRaw(raw);
  NavInputs in;
  in.dt = dt;
  in.line = reading_;
  in.obstacleLeft = digitalRead(kObstacleLeft) == OBSTACLE_ACTIVE;
  in.obstacleRight = digitalRead(kObstacleRight) == OBSTACLE_ACTIVE;
  in.heading = heading_;
  in.distance = odometry_.distance();
  const uint32_t previous = nav_.transitions();
  const Twist cmd = nav_.update(in);
  if (!nav_.running()) {
    disableDrive();
    mode_ = Mode::Stopped;
  }
  if (previous != nav_.transitions()) {
    Serial.print("nav: ");
    Serial.print(navStateName(nav_.state()));
    Serial.print(" (");
    Serial.print(nav_.reason());
    Serial.print(")");
    printPose();
  }
  if (mode_ == Mode::Stopped) {
    Serial.print("nav stopped: ");
    Serial.print(navStateName(nav_.state()));
    Serial.print(", ");
    Serial.print(nav_.corners());
    Serial.print(" corners, ");
    Serial.print(nav_.detours());
    Serial.println(" detours");
    return Twist();
  }
  return cmd;
}

void DirectLineTracker::updateDrive(uint32_t now, uint32_t left, uint32_t right) {
  const float dt = (now - lastSpeed_) / 1000.0f;
  const float vl = wheelTravel(left, speedLeft_) / dt;
  const float vr = wheelTravel(right, speedRight_) / dt;
  speedLeft_ = left;
  speedRight_ = right;
  lastSpeed_ = now;
  if (!active()) return;
  // Re-read the live watchdog after sensor I/O and immediately before actuation.
  if (!motionPermitted_()) {
    stop();
    return;
  }
  if (digitalRead(kFault) == LOW) {
    stop();
    Serial.println("DRV8833 FAULT: outputs disabled");
    return;
  }
  drive(kLeftIn1, kLeftIn2, leftPI_.update(targets_.left, vl, dt));
  drive(kRightIn1, kRightIn2, rightPI_.update(targets_.right, vr, dt));
  if (!enabled_) {
    digitalWrite(kSleep, HIGH);
    enabled_ = true;
  }
}

void DirectLineTracker::update() {
  if (active() && !motionPermitted_()) stop();
  float gx, gy, gz;
  if (imuReady_ && IMU.gyroscopeAvailable() && IMU.readGyroscope(gx, gy, gz)) {
    const uint32_t us = micros();
    const float dt = (us - lastGyroUs_) / 1e6f;
    lastGyroUs_ = us;
    if (mode_ == Mode::Waiting) gyroBias_.add(gz);
    else heading_ = wrapAngle(heading_ + degToRad((gz - gyroBias_.bias()) * dt));
  }
  const uint32_t now = millis();
  if (now - lastTick_ < 5) return;
  const float dt = (now - lastTick_) / 1000.0f;
  lastTick_ = now;
  noInterrupts();
  const uint32_t left = leftCount_, right = rightCount_;
  interrupts();
  odometry_.updateWithGyro(wheelTravel(left, tickLeft_), wheelTravel(right, tickRight_),
                           wrapAngle(heading_ - tickHeading_));
  tickLeft_ = left;
  tickRight_ = right;
  tickHeading_ = heading_;
  if (active()) {
    uint16_t raw[4];
    readLine(raw);
    const Twist cmd = mode_ == Mode::Calibrating ? calibrate(raw) : navigate(raw, dt);
    targets_ = toWheels(limitTwist(cmd, kTrack, kMaxWheelSpeed), kTrack);
  }
  if (now - lastSpeed_ >= 10) updateDrive(now, left, right);
  digitalWrite(LEDG, mode_ == Mode::Running ? LOW : HIGH);
  digitalWrite(LEDB, mode_ == Mode::Waiting || mode_ == Mode::Calibrating ? LOW : HIGH);
  digitalWrite(LEDR, mode_ == Mode::Stopped && nav_.state() != NavState::Finished ? LOW : HIGH);
  if (mode_ == Mode::Running && now - lastReport_ >= 1000) {
    lastReport_ = now;
    Serial.print(navStateName(nav_.state()));
    Serial.print(" line=");
    Serial.print(reading_.onLine ? reading_.position * 100.0f : NAN, 1);
    Serial.print(" cm");
    printPose();
  }
}

}  // namespace mech368
