#include <Arduino.h>
#include <MotionSafety.h>

mech368::MotionSafety safety;

void setup() {
  // TODO: Verify driver, safe electrical defaults and board wiring before
  // implementing any motor- or battery-related I/O.
}
//setup 
//test test
void loop() {
  // TODO: Replace placeholders with bounded, nonblocking BLE input and a
  // local APDS-9960 gesture event. Never pass cached packets as new commands.
  const bool linkUp = false;
  const auto packet = mech368::Packet::None;
  const bool localRecovery = false;
  const uint32_t now = static_cast<uint32_t>(millis());
  safety.update(now, linkUp, packet, localRecovery);

  if (!safety.motionAllowed(now)) {
    // TODO: Apply a verified zero motor command and update status indication.
    return;
  }

  // TODO: Gate all motor paths here: differential steering, line following,
  // obstacle avoidance and pose tracking. No actuator output in this starter.
}
