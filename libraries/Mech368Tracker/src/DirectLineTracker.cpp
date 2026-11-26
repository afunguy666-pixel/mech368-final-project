#include "DirectLineTracker.h"
#include "DriveCalibration.h"
#include <Arduino.h>
#include <USB/PluggableUSBSerial.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>

namespace mech368 {
using namespace line;

namespace {
// The normal Nano USB Serial.write may wait for a terminal indefinitely.
// Stage one debug line, then make one best-effort CDC transfer.
class DebugLine : public Print {
 public:
  size_t write(uint8_t c) override {
    if (c == '\n') {
      line_[size_++] = c;
      uint32_t sent;
      _SerialUSB.send_nb(line_, size_, &sent);
      size_ = 0;
    } else if (size_ < sizeof(line_) - 1) {
      line_[size_++] = c;
    }
    return 1;
  }
 private:
  uint8_t line_[64]; // one full-speed endpoint packet, reserve its newline
  uint32_t size_ = 0;
} debugLine;
constexpr int kLeftIn1 = D4, kLeftIn2 = D5, kRightIn1 = D6, kRightIn2 = D7;
constexpr int kSleep = D8, kFault = A6;
constexpr int kLeftA = D2, kLeftB = D3, kRightA = D9, kRightB = D10;
constexpr int kObstacleLeft = D12, kObstacleRight = D11;  // LM393 active low
constexpr int kLinePins[] = {A0, A1, A2, A3};            // right to left
constexpr float kLineY[] = {-0.030f, -0.010f, 0.010f, 0.030f};
constexpr bool BLACK_IS_HIGH = true;
constexpr int OBSTACLE_ACTIVE = LOW;
constexpr uint16_t MIN_CAL_SPAN = 200;
constexpr float kTrack = calibration::trackMetres;
// The 14.9 mm caster spacer levels the chassis. A probe normal to the
// clamp samples the floor directly below its centreline.
constexpr float kSensorAhead = 0.07575f;
constexpr float kTapeWidth = 0.050f, kMaxWheelSpeed = 0.45f;
constexpr float kMotorTopSpeed = 0.95f;

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

void DirectLineTracker::sampleInfrared(uint16_t* out) const {
  if (active()) {
    for (int i = 0; i < 4; ++i) out[i] = raw_[i];
  } else {
    readLine(out);
  }
}

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
  if (!imuReady_) debugLine.println("IMU init failed");
  if (!gestureReady_) debugLine.println("APDS init failed");
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
    debugLine.println("DRV8833 FAULT: start inhibited");
    return;
  }
  if (!gyroBias_.ready(50)) {
    debugLine.println("gesture UP ignored: gyro bias not measured yet, keep still");
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
    debugLine.println("gesture UP: calibrating line sensors");
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
  debugLine.print("nav: ");
  debugLine.print(navStateName(nav_.state()));
  debugLine.print(" (");
  debugLine.print(nav_.reason());
  debugLine.println(")");
}

Twist DirectLineTracker::calibrate(const uint16_t* raw) {
  sensors_.calibrate(raw);
  const float angles[] = {degToRad(40), degToRad(-40), 0};
  const float error = wrapAngle(calStart_ + angles[calPhase_] - heading_);
  if (fabsf(error) < degToRad(2) && ++calPhase_ == 3) {
    debugLine.print("calibration min/max:");
    for (int i = 0; i < 4; ++i) {
      debugLine.print(' ');
      debugLine.print(sensors_.calibrationMin(i));
      debugLine.print('/');
      debugLine.print(sensors_.calibrationMax(i));
    }
    debugLine.println();
    haveCalStart_ = false;
    calibrated_ = sensors_.calibrationValid(MIN_CAL_SPAN);
    if (calibrated_) {
      startNavigation();
      if (reading_.value[1] + reading_.value[2] < reading_.value[0] + reading_.value[3])
        debugLine.println("warning: inner sensors read lighter than outer - check sensor polarity");
    } else {
      stop();
      debugLine.println("calibration FAILED: a sensor never saw both tape and floor");
    }
    return Twist();
  }
  float omega = clampf(4.0f * error, -2.0f, 2.0f);
  if (fabsf(omega) < 0.8f) omega = omega < 0 ? -0.8f : 0.8f;
  return Twist(0, omega);
}

void DirectLineTracker::printPose() {
  debugLine.print(" at (");
  debugLine.print(odometry_.pose().x, 3);
  debugLine.print(", ");
  debugLine.print(odometry_.pose().y, 3);
  debugLine.print(") m, heading ");
  debugLine.print(radToDeg(heading_), 1);
  debugLine.println(" deg");
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
    debugLine.print("nav: ");
    debugLine.print(navStateName(nav_.state()));
    debugLine.print(" (");
    debugLine.print(nav_.reason());
    debugLine.print(")");
    printPose();
  }
  if (mode_ == Mode::Stopped) {
    debugLine.print("nav stopped: ");
    debugLine.print(navStateName(nav_.state()));
    debugLine.print(", ");
    debugLine.print(nav_.corners());
    debugLine.print(" corners, ");
    debugLine.print(nav_.detours());
    debugLine.println(" detours");
    return Twist();
  }
  return cmd;
}

void DirectLineTracker::updateDrive(uint32_t now, uint32_t left, uint32_t right) {
  const float dt = (now - lastSpeed_) / 1000.0f;
  const float vl = calibration::left.travel(left, speedLeft_) / dt;
  const float vr = calibration::right.travel(right, speedRight_) / dt;
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
    debugLine.println("DRV8833 FAULT: outputs disabled");
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
    if (!gyroBias_.ready(50)) {
      const uint32_t l = leftCount_, r = rightCount_;
      if (l != biasLeft_ || r != biasRight_ || fabsf(gz) > 5.0f) gyroBias_.reset();
      else gyroBias_.add(gz);
      biasLeft_ = l;
      biasRight_ = r;
    } else {
      heading_ = wrapAngle(heading_ + degToRad((gz - gyroBias_.bias()) * dt));
    }
  }
  const uint32_t now = millis();
  if (now - lastTick_ < 5) return;
  const float dt = (now - lastTick_) / 1000.0f;
  lastTick_ = now;
  noInterrupts();
  const uint32_t left = leftCount_, right = rightCount_;
  interrupts();
  odometry_.updateWithGyro(calibration::left.travel(left, tickLeft_), calibration::right.travel(right, tickRight_),
                           wrapAngle(heading_ - tickHeading_));
  tickLeft_ = left;
  tickRight_ = right;
  tickHeading_ = heading_;
  if (active()) {
    readLine(raw_);
    const Twist cmd = mode_ == Mode::Calibrating ? calibrate(raw_) : navigate(raw_, dt);
    targets_ = toWheels(limitTwist(cmd, kTrack, kMaxWheelSpeed), kTrack);
  }
  if (now - lastSpeed_ >= 10) updateDrive(now, left, right);
  digitalWrite(LEDG, mode_ == Mode::Running ? LOW : HIGH);
  digitalWrite(LEDB, mode_ == Mode::Waiting || mode_ == Mode::Calibrating ? LOW : HIGH);
  digitalWrite(LEDR, mode_ == Mode::Stopped && nav_.state() != NavState::Finished ? LOW : HIGH);
  if (mode_ == Mode::Running && now - lastReport_ >= 1000) {
    lastReport_ = now;
    debugLine.print(navStateName(nav_.state()));
    debugLine.print(" line=");
    debugLine.print(reading_.onLine ? reading_.position * 100.0f : NAN, 1);
    debugLine.print(" cm");
    printPose();
  }
}

}  // namespace mech368
