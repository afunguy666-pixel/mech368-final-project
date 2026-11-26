#ifndef MECH368_DIRECT_LINE_TRACKER_H
#define MECH368_DIRECT_LINE_TRACKER_H

#include <Mech368Line.h>

namespace mech368 {

class DirectLineTracker {
 public:
  explicit DirectLineTracker(bool (*motionPermitted)()) : motionPermitted_(motionPermitted) {}
  void begin(void (*leftEncoderIsr)(), void (*rightEncoderIsr)());
  void leftEdge();
  void rightEdge();
  int readGesture();
  void start();
  void stop();
  void update();
  bool outputsEnabled() const { return enabled_; }

 private:
  enum class Mode { Waiting, Calibrating, Running, Stopped };
  bool active() const { return mode_ == Mode::Calibrating || mode_ == Mode::Running; }
  void disableDrive();
  void startNavigation();
  line::Twist calibrate(const uint16_t* raw);
  line::Twist navigate(const uint16_t* raw, float dt);
  void printPose();
  void updateDrive(uint32_t now, uint32_t left, uint32_t right);

  bool (*motionPermitted_)();
  Mode mode_ = Mode::Waiting;
  bool enabled_ = false, calibrated_ = false, imuReady_ = false, gestureReady_ = false;
  bool haveCalStart_ = false;
  line::LineSensorArray sensors_;
  line::LineNavigator nav_;
  line::Odometry odometry_;
  line::GyroBias gyroBias_;
  line::WheelSpeedPI leftPI_, rightPI_;
  line::LineReading reading_;
  line::WheelSpeeds targets_;
  volatile uint32_t leftCount_ = 0, rightCount_ = 0;
  uint32_t tickLeft_ = 0, tickRight_ = 0, speedLeft_ = 0, speedRight_ = 0;
  uint32_t lastTick_ = 0, lastSpeed_ = 0, lastGyroUs_ = 0, lastReport_ = 0;
  float heading_ = 0, tickHeading_ = 0, calStart_ = 0;
  unsigned calPhase_ = 0;
};

}  // namespace mech368
#endif
