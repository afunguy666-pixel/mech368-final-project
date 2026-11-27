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
  // Start AUTO: line sensor calibration sweep (first time), then navigation.
  bool start();
  // Start MANUAL: wheel targets follow setManualTwist(). Same drive path,
  // watchdog and DRV8833 fault checks as AUTO.
  bool startManual();
  // Newest operator twist. Ignored unless MANUAL is running.
  void setManualTwist(const line::Twist& twist) { manualCmd_ = twist; }
  void stop();
  void update();
  // One best-effort, non-blocking USB debug line.
  void log(const char* text);
  // false: the sketch drives the RGB LED itself.
  bool ownLeds = true;
  bool outputsEnabled() const { return enabled_; }
  const line::Odometry& odometry() const { return odometry_; }
  const line::GyroBias& gyroBias() const { return gyroBias_; }
  uint32_t sampleMs() const { return lastTick_; }
  uint32_t leftCount() const { return tickLeft_; }
  uint32_t rightCount() const { return tickRight_; }
  bool calibrating() const { return mode_ == Mode::Calibrating; }
  bool running() const { return mode_ == Mode::Running; }
  bool manual() const { return mode_ == Mode::Manual; }
  bool active() const { return mode_ == Mode::Calibrating || mode_ == Mode::Running ||
                               mode_ == Mode::Manual; }
  // Navigation ended by itself on lost line / blocked (not finished, not a stop).
  bool navFault() const;
  line::NavState navState() const { return nav_.state(); }
  const line::LineReading& lineReading() const { return reading_; }
  void sampleInfrared(uint16_t* out) const;

 private:
  enum class Mode { Waiting, Calibrating, Running, Stopped, Manual };
  bool startAllowed(const char* what);
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
  line::Twist manualCmd_;
  volatile uint32_t leftCount_ = 0, rightCount_ = 0;
  uint32_t tickLeft_ = 0, tickRight_ = 0, speedLeft_ = 0, speedRight_ = 0;
  uint32_t biasLeft_ = 0, biasRight_ = 0;
  uint16_t raw_[4] = {};
  uint32_t lastTick_ = 0, lastSpeed_ = 0, lastGyroUs_ = 0, lastReport_ = 0;
  float heading_ = 0, tickHeading_ = 0, calStart_ = 0;
  unsigned calPhase_ = 0;
};

}  // namespace mech368
#endif
