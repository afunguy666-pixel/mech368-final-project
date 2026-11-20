#include <MotionSafety.h>
#include <cassert>

using mech368::MotionSafety;
using mech368::Packet;

int main() {
  static_assert(MotionSafety::kTimeoutMs <= 500, "watchdog exceeds 500 ms");
  constexpr uint32_t timeout = MotionSafety::kTimeoutMs;
  MotionSafety gate;

  gate.update(0, true, Packet::None);
  assert(!gate.motionAllowed(0));
  gate.update(1, true, Packet::Invalid);
  assert(!gate.motionAllowed(1));
  gate.update(2, true, Packet::Motion);
  assert(gate.motionAllowed(2));
  gate.update(2 + timeout - 1, true, Packet::Invalid);
  assert(gate.motionAllowed(2 + timeout - 1));
  assert(!gate.motionAllowed(2 + timeout));
  gate.update(2 + timeout, true, Packet::None);
  assert(!gate.motionAllowed(2 + timeout));
  gate.update(3 + timeout, true, Packet::Motion);
  assert(gate.motionAllowed(3 + timeout));
  gate.update(4 + timeout, false, Packet::Motion);
  assert(!gate.motionAllowed(4 + timeout));
  gate.update(5 + timeout, true, Packet::None);
  assert(!gate.motionAllowed(5 + timeout));

  MotionSafety stopped;
  stopped.update(1, true, Packet::Motion);
  stopped.update(2, true, Packet::EmergencyStop, true);
  assert(stopped.emergencyStopLatched());
  assert(!stopped.motionAllowed(2));
  stopped.update(3, false, Packet::None);
  stopped.update(4, true, Packet::Motion);
  assert(stopped.emergencyStopLatched());
  assert(!stopped.motionAllowed(4));
  stopped.update(5, true, Packet::Motion, true);
  assert(!stopped.emergencyStopLatched());
  assert(!stopped.motionAllowed(5));
  stopped.update(6, true, Packet::Invalid);
  assert(!stopped.motionAllowed(6));
  stopped.update(7, true, Packet::Motion);
  assert(stopped.motionAllowed(7));

  MotionSafety rollover;
  const uint32_t beforeWrap = UINT32_MAX - 10U;
  rollover.update(beforeWrap, true, Packet::Motion);
  assert(rollover.motionAllowed(static_cast<uint32_t>(beforeWrap + timeout - 1)));
  assert(!rollover.motionAllowed(static_cast<uint32_t>(beforeWrap + timeout)));
}
