// Differential-drive kinematics, odometry, wheel speed control and IMU
// helpers. Portable C++11, no Arduino dependencies, host-tested in
// tests/line_nav_test.cpp. Units: metres, seconds, radians (CCW positive).
#ifndef MECH368_DRIVE_MATH_H
#define MECH368_DRIVE_MATH_H

#include <math.h>
#include <stdint.h>

namespace mech368 {
namespace line {

constexpr float kPi = 3.14159265358979f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline float signf(float v) { return v > 0.0f ? 1.0f : (v < 0.0f ? -1.0f : 0.0f); }
inline float degToRad(float deg) { return deg * kPi / 180.0f; }
inline float radToDeg(float rad) { return rad * 180.0f / kPi; }

// Wrap to (-pi, pi]. Use the difference of two headings, wrapped, for errors.
inline float wrapAngle(float a) {
  while (a > kPi) a -= 2.0f * kPi;
  while (a <= -kPi) a += 2.0f * kPi;
  return a;
}

// Velocity twist [v_c, omega] from the handout.
struct Twist {
  float v;      // m/s, forward
  float omega;  // rad/s, CCW (left turn) positive
  Twist(float v_ = 0.0f, float omega_ = 0.0f) : v(v_), omega(omega_) {}
};

// Linear speed of each wheel's rim (m/s).
struct WheelSpeeds {
  float left, right;
  WheelSpeeds(float l = 0.0f, float r = 0.0f) : left(l), right(r) {}
};

// Handout eq. (4) and (6): v = (vl + vr) / 2, omega = (vr - vl) / W.
inline WheelSpeeds toWheels(const Twist& t, float track) {
  return WheelSpeeds(t.v - t.omega * track / 2.0f, t.v + t.omega * track / 2.0f);
}
inline Twist toTwist(const WheelSpeeds& w, float track) {
  return Twist((w.left + w.right) / 2.0f, (w.right - w.left) / track);
}

// Limit a twist so neither wheel exceeds maxWheel (m/s). The turn rate is
// kept and forward speed is given up first: on a line follower the steering
// correction matters more than speed, and the robot slows into a bend instead
// of running wide.
inline Twist limitTwist(Twist t, float track, float maxWheel) {
  const float turn = fabsf(t.omega) * track / 2.0f;
  if (turn >= maxWheel) return Twist(0.0f, signf(t.omega) * 2.0f * maxWheel / track);
  t.v = clampf(t.v, -(maxWheel - turn), maxWheel - turn);
  return t;
}

struct Pose {
  float x, y, heading;
  Pose(float x_ = 0.0f, float y_ = 0.0f, float h = 0.0f) : x(x_), y(y_), heading(h) {}
};

// Dead reckoning from wheel travel. Heading can come from the encoders
// (handout eq. 2a) or, better on a slipping robot, from the gyro.
class Odometry {
 public:
  void reset(const Pose& p) { pose_ = p; distance_ = 0.0f; }
  // dl, dr: wheel travel since the last call (m).
  void update(float dl, float dr, float track) { advance((dl + dr) / 2.0f, (dr - dl) / track); }
  // Encoder distance, gyro heading change (rad) over the same interval.
  void updateWithGyro(float dl, float dr, float dHeading) { advance((dl + dr) / 2.0f, dHeading); }
  const Pose& pose() const { return pose_; }
  // Signed path length since reset (m): reverse motion counts negative.
  float distance() const { return distance_; }

 private:
  void advance(float ds, float dh) {
    const float mid = pose_.heading + dh / 2.0f;  // midpoint (2nd-order) integration
    pose_.x += ds * cosf(mid);
    pose_.y += ds * sinf(mid);
    pose_.heading = wrapAngle(pose_.heading + dh);
    distance_ += ds;
  }
  Pose pose_;
  float distance_ = 0.0f;
};

// Running mean of a rate gyro axis while the robot is known to be still.
// Samples that differ from the mean by more than rejectDps are ignored so a
// nudge during calibration does not poison the estimate.
class GyroBias {
 public:
  explicit GyroBias(float rejectDps = 3.0f) : reject_(rejectDps) {}
  void reset() { sum_ = 0.0f; count_ = 0; }
  void add(float rateDps) {
    if (count_ >= 20 && fabsf(rateDps - bias()) > reject_) return;
    sum_ += rateDps;
    ++count_;
  }
  uint32_t samples() const { return count_; }
  bool ready(uint32_t minSamples = 100) const { return count_ >= minSamples; }
  float bias() const { return count_ ? sum_ / static_cast<float>(count_) : 0.0f; }

 private:
  float reject_;
  float sum_ = 0.0f;
  uint32_t count_ = 0;
};

// Roll and pitch (rad) from a static accelerometer reading (any unit).
// roll > 0: right edge down; pitch > 0: front (+x) edge down.
inline void rollPitchFromAccel(float ax, float ay, float az, float& roll, float& pitch) {
  roll = atan2f(ay, az);
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
}

// Wheel speed loop: feed-forward on the target plus PI on the encoder error.
// Output is a signed duty -1..1. The integral is clamped (anti-windup) and
// cleared whenever the target is zero so the wheel stops dead.
class WheelSpeedPI {
 public:
  float feedForward = 1.0f / 0.48f;  // duty per (m/s): 1 / top speed
  float kp = 1.0f, ki = 6.0f;        // duty per (m/s), duty per m
  float integralLimit = 0.08f;       // m
  void reset() { integral_ = 0.0f; }
  float update(float target, float measured, float dt) {
    if (target == 0.0f) {
      integral_ = 0.0f;
      return 0.0f;
    }
    const float error = target - measured;
    integral_ = clampf(integral_ + error * dt, -integralLimit, integralLimit);
    return clampf(feedForward * target + kp * error + ki * integral_, -1.0f, 1.0f);
  }

 private:
  float integral_ = 0.0f;
};

}  // namespace line
}  // namespace mech368
#endif
