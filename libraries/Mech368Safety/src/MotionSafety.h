#ifndef MECH368_MOTION_SAFETY_H
#define MECH368_MOTION_SAFETY_H

#include <stdint.h>

namespace mech368 {

// 只有适配层可以把收到的报文判定为新鲜且有效。
enum class Packet : uint8_t { None, Invalid, Motion, EmergencyStop };

class MotionSafety {
 public:
  // 从课程要求的 500 ms 预算中预留 50 ms，用于轮询和输出延迟。
  static constexpr uint32_t kTimeoutMs = 450;

  void update(uint32_t now, bool linkUp, Packet packet,
              bool localRecoveryGestureEdge = false) {
    linkUp_ = linkUp;
    if (!linkUp_ || (hasCommand_ && expired(now))) {
      hasCommand_ = false;
    }

    // 经过校验的急停总是优先于同时发生的恢复手势。
    if (packet == Packet::EmergencyStop) {
      latched_ = true;
      hasCommand_ = false;
      return;
    }

    if (latched_) {
      if (localRecoveryGestureEdge) latched_ = false;
      // 即使运动指令恰好在恢复的那次轮询中到达，也将其丢弃。
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
