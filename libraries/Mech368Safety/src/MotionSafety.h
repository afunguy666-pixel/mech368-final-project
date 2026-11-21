#ifndef MECH368_MOTION_SAFETY_H
#define MECH368_MOTION_SAFETY_H

#include <stdint.h>

namespace mech368 {

// Only the adapter may classify a received frame as fresh and valid.
enum class Packet : uint8_t { None, Invalid, Motion, EmergencyStop };

class MotionSafety {
 public:
  // Reserve 50 ms of the course's 500 ms budget for polling and output latency.
  static constexpr uint32_t kTimeoutMs = 450;

  void update(uint32_t now, bool linkUp, Packet packet,
              bool localRecoveryGestureEdge = false) {
    linkUp_ = linkUp;
    if (!linkUp_ || (hasCommand_ && expired(now))) {
      hasCommand_ = false;
    }

    // A validated E-stop always wins over a simultaneous recovery gesture.
    if (packet == Packet::EmergencyStop) {
      latched_ = true;
      hasCommand_ = false;
      return;
    }

    if (latched_) {
      if (localRecoveryGestureEdge) latched_ = false;
      // Discard a motion command even if it arrives during the recovery poll.
      return;
    }

    if (linkUp_ && packet == Packet::Motion) {
      receivedAt_ = now;
      hasCommand_ = true;
    }
  }

  bool motionAllowed(uint32_t now) const {
    return linkUp_ && !latched_ && hasCommand_ && !expired(now);
  }

  bool emergencyStopLatched() const { return latched_; }

 private:
  bool expired(uint32_t now) const {
    return static_cast<uint32_t>(now - receivedAt_) >= kTimeoutMs;
  }

  uint32_t receivedAt_ = 0;
  bool linkUp_ = false;
  bool hasCommand_ = false;
  bool latched_ = false;
};

}  // namespace mech368
#endif
