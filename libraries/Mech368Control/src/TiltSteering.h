// Handheld tilt to (v, omega). Portable C++11, host-tested in
// tests/control_test.cpp. The handheld sketch feeds accelerometer samples
// (g) and sends the result in the protocol's Motion frame fields.
//
// Conventions (same as line::rollPitchFromAccel):
//   pitch > 0: front (+x, USB end away from you) edge down
//   roll  > 0: right edge down
// With the defaults, tipping the front down drives forward and tipping the
// right edge down turns right (clockwise, omega < 0). Flip pitchSign or
// rollSign if the board is held another way.
//
// Fail-safe rules:
//   - no IMU, still zeroing, a sample older than staleMs, or an implausible
//     sample (|a| far from 1 g, not finite) -> output exactly 0 at once;
//   - after zeroing or any loss of fresh data the output stays 0 until both
//     axes have been seen inside the deadband ("neutral re-arm"), so a board
//     that is already tilted when data returns does not lurch the robot.
#ifndef MECH368_TILT_STEERING_H
#define MECH368_TILT_STEERING_H

#include <DriveMath.h>
#include <math.h>
#include <stdint.h>
#include "SlewLimiter.h"

namespace mech368 {
namespace control {

struct TiltConfig {
  float deadbandDeg = 5.0f;     // no motion inside +-deadband about the zero
  float fullScaleDeg = 25.0f;   // tilt for the maximum command
  float maxLinear = 0.25f;      // m/s at full scale (bench value, not track-approved)
  float maxAngular = 2.0f;      // rad/s at full scale
  float linearSlew = 0.6f;      // m/s per s
  float angularSlew = 6.0f;     // rad/s per s
  float pitchSign = 1.0f;       // +1: front edge down drives forward
  float rollSign = -1.0f;       // -1: right edge down turns right
  uint32_t staleMs = 100;       // accelerometer runs at 100 Hz
  uint16_t zeroSamples = 50;    // ~0.5 s of still samples
  float stillDeg = 1.5f;        // max deviation from the zeroing mean
  float maxZeroTiltDeg = 30.0f; // refuse a zero further than this from flat
  float filterAlpha = 0.35f;    // EMA weight of each new sample
  float minG = 0.8f, maxG = 1.2f;  // plausible magnitude, otherwise skip sample
};

enum class TiltState : uint8_t { NoImu, Zeroing, WaitNeutral, Active, Stale };

inline const char* tiltStateName(TiltState s) {
  switch (s) {
    case TiltState::NoImu: return "no-imu";
    case TiltState::Zeroing: return "zeroing";
    case TiltState::WaitNeutral: return "wait-neutral";
    case TiltState::Active: return "active";
    case TiltState::Stale: return "stale";
  }
  return "?";
}

class TiltSteering {
 public:
  explicit TiltSteering(const TiltConfig& cfg = TiltConfig())
      : cfg_(cfg), linear_(cfg.linearSlew), angular_(cfg.angularSlew) {}

  const TiltConfig& config() const { return cfg_; }
  void setImuReady(bool ready) {
    imuReady_ = ready;
    if (!ready) dropOutput();
  }
  // Start a new zero measurement. Output is 0 until it completes.
  void requestZero() {
    zeroed_ = false;
    count_ = 0;
    dropOutput();
  }

  // One accelerometer sample in g at time ms (millis()).
  void sample(uint32_t ms, float ax, float ay, float az) {
    if (!imuReady_) return;
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az)) return;
    const float g = sqrtf(ax * ax + ay * ay + az * az);
    if (g < cfg_.minG || g > cfg_.maxG) return;  // shaking or bad read
    float roll, pitch;
    line::rollPitchFromAccel(ax, ay, az, roll, pitch);
    roll = line::radToDeg(roll);
    pitch = line::radToDeg(pitch);
    lastSampleMs_ = ms;
    haveSample_ = true;
    if (!zeroed_) {
      zeroStep(roll, pitch);
      return;
    }
    const float r = roll - zeroRoll_, p = pitch - zeroPitch_;
    if (!filtered_) {
      fRoll_ = r;
      fPitch_ = p;
      filtered_ = true;
    } else {
      fRoll_ += cfg_.filterAlpha * (r - fRoll_);
      fPitch_ += cfg_.filterAlpha * (p - fPitch_);
    }
    if (!armed_ && fabsf(fRoll_) < cfg_.deadbandDeg && fabsf(fPitch_) < cfg_.deadbandDeg)
      armed_ = true;
  }

  // Call every loop. Returns the slew-limited command.
  line::Twist update(uint32_t ms) {
    const float dt = haveUpdate_ ? static_cast<uint32_t>(ms - lastUpdateMs_) / 1000.0f : 0.0f;
    lastUpdateMs_ = ms;
    haveUpdate_ = true;
    if (!fresh(ms)) {
      dropOutput();
      return line::Twist();
    }
    float v = 0.0f, w = 0.0f;
    if (zeroed_ && armed_) {
      v = cfg_.pitchSign * shape(fPitch_) * cfg_.maxLinear;
      w = cfg_.rollSign * shape(fRoll_) * cfg_.maxAngular;
    }
    return line::Twist(linear_.step(v, dt), angular_.step(w, dt));
  }

  TiltState state(uint32_t ms) const {
    if (!imuReady_) return TiltState::NoImu;
    if (!zeroed_) return TiltState::Zeroing;
    if (!fresh(ms)) return TiltState::Stale;
    return armed_ ? TiltState::Active : TiltState::WaitNeutral;
  }
  bool zeroed() const { return zeroed_; }
  float rollDeg() const { return fRoll_; }   // relative to the zero, filtered
  float pitchDeg() const { return fPitch_; }
  float zeroRollDeg() const { return zeroRoll_; }
  float zeroPitchDeg() const { return zeroPitch_; }

  // Signed fraction 0..1 of full scale outside the deadband.
  float shape(float deg) const {
    const float a = fabsf(deg);
    if (a <= cfg_.deadbandDeg) return 0.0f;
    const float span = cfg_.fullScaleDeg - cfg_.deadbandDeg;
    const float f = span > 0.0f ? (a - cfg_.deadbandDeg) / span : 1.0f;
    return line::signf(deg) * (f > 1.0f ? 1.0f : f);
  }

 private:
  bool fresh(uint32_t ms) const {
    return imuReady_ && zeroed_ && haveSample_ &&
           static_cast<uint32_t>(ms - lastSampleMs_) < cfg_.staleMs;
  }
  void dropOutput() {
    linear_.reset();
    angular_.reset();
    armed_ = false;
    filtered_ = false;
  }
  void zeroStep(float roll, float pitch) {
    if (count_ > 0) {
      const float mr = sumRoll_ / count_, mp = sumPitch_ / count_;
      if (fabsf(roll - mr) > cfg_.stillDeg || fabsf(pitch - mp) > cfg_.stillDeg) count_ = 0;
    }
    if (count_ == 0) sumRoll_ = sumPitch_ = 0.0f;
    sumRoll_ += roll;
    sumPitch_ += pitch;
    if (++count_ < cfg_.zeroSamples) return;
    const float mr = sumRoll_ / count_, mp = sumPitch_ / count_;
    count_ = 0;
    if (fabsf(mr) > cfg_.maxZeroTiltDeg || fabsf(mp) > cfg_.maxZeroTiltDeg) return;
    zeroRoll_ = mr;
    zeroPitch_ = mp;
    zeroed_ = true;
    dropOutput();
  }

  TiltConfig cfg_;
  SlewLimiter linear_, angular_;
  bool imuReady_ = false, zeroed_ = false, armed_ = false, filtered_ = false;
  bool haveSample_ = false, haveUpdate_ = false;
  uint32_t lastSampleMs_ = 0, lastUpdateMs_ = 0;
  uint16_t count_ = 0;
  float sumRoll_ = 0.0f, sumPitch_ = 0.0f;
  float zeroRoll_ = 0.0f, zeroPitch_ = 0.0f, fRoll_ = 0.0f, fPitch_ = 0.0f;
};

}  // namespace control
}  // namespace mech368
#endif
