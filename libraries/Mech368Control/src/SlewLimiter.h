// Rate limiter for a commanded value. Portable C++11, host-tested.
#ifndef MECH368_SLEW_LIMITER_H
#define MECH368_SLEW_LIMITER_H

#include <stdint.h>

namespace mech368 {
namespace control {

class SlewLimiter {
 public:
  // ratePerSecond: largest change of the output per second (> 0).
  explicit SlewLimiter(float ratePerSecond = 1.0f) : rate_(ratePerSecond) {}
  void setRate(float ratePerSecond) { rate_ = ratePerSecond; }
  // Jump to a value without limiting, e.g. 0 on a fault.
  void reset(float value = 0.0f) { value_ = value; }
  // dtSeconds is clamped to [0, 0.1] so a stalled loop cannot cause a jump.
  float step(float target, float dtSeconds) {
    if (!(dtSeconds > 0.0f)) return value_;
    if (dtSeconds > 0.1f) dtSeconds = 0.1f;
    const float limit = rate_ * dtSeconds;
    const float delta = target - value_;
    if (delta > limit) value_ += limit;
    else if (delta < -limit) value_ -= limit;
    else value_ = target;
    return value_;
  }
  float value() const { return value_; }

 private:
  float rate_;
  float value_ = 0.0f;
};

}  // namespace control
}  // namespace mech368
#endif
