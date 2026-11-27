// IDLE / MANUAL / AUTO selection on the robot. Portable C++11, host-tested
// in tests/control_test.cpp. The robot sketch maps APDS-9960 gestures on the
// car to Gesture values, calls update() once per loop and performs the
// returned action on the tracker. The tracker (and its MotionSafety gate)
// remains the authority on whether the bridge is actually driven.
//
//   LEFT  select MANUAL  (stops first if anything is driving)
//   RIGHT select AUTO    (stops first if anything is driving)
//   UP    start the selected mode (refused if nothing is selected, the link
//         is not permitting motion, or MANUAL is selected and the handheld
//         is not at neutral)
//   DOWN  stop to IDLE
// While the E-stop latch is set any gesture only clears the latch (handled by
// MotionSafety); it is consumed here and never selects or starts anything.
// Selecting never starts. Reconnecting, a fresh heartbeat or clearing the
// latch never starts. Only a separate UP starts.
#ifndef MECH368_DRIVE_MODE_SWITCH_H
#define MECH368_DRIVE_MODE_SWITCH_H

#include <stdint.h>

namespace mech368 {
namespace control {

enum class DriveMode : uint8_t { Idle = 0, Manual = 1, Auto = 2 };
enum class Gesture : uint8_t { None, Up, Down, Left, Right };
enum class ModeAction : uint8_t { None, Stop, StartAuto, StartManual };

inline const char* driveModeName(DriveMode m) {
  switch (m) {
    case DriveMode::Idle: return "IDLE";
    case DriveMode::Manual: return "MANUAL";
    case DriveMode::Auto: return "AUTO";
  }
  return "?";
}

struct ModeInputs {
  Gesture gesture = Gesture::None;
  bool latched = false;          // E-stop latch state before this gesture
  bool motionPermitted = false;  // MotionSafety: fresh valid packets, not latched
  bool driving = false;          // tracker active (calibrating, following or manual)
  bool manualNeutral = false;    // newest accepted remote command is exactly (0, 0)
};

class DriveModeSwitch {
 public:
  ModeAction update(const ModeInputs& in) {
    reason_ = "";
    if (in.gesture == Gesture::None) return ModeAction::None;
    if (in.latched) {
      reason_ = "gesture used to clear the E-stop latch only";
      return ModeAction::None;
    }
    switch (in.gesture) {
      case Gesture::Down:
        reason_ = "DOWN: stop";
        return ModeAction::Stop;
      case Gesture::Left:
      case Gesture::Right:
        selected_ = in.gesture == Gesture::Left ? DriveMode::Manual : DriveMode::Auto;
        reason_ = in.gesture == Gesture::Left ? "LEFT: MANUAL selected" : "RIGHT: AUTO selected";
        return in.driving ? ModeAction::Stop : ModeAction::None;
      case Gesture::Up:
        if (in.driving) { reason_ = "UP ignored: already driving, DOWN first"; return ModeAction::None; }
        if (selected_ == DriveMode::Idle) { reason_ = "UP ignored: select MANUAL (LEFT) or AUTO (RIGHT) first"; return ModeAction::None; }
        if (!in.motionPermitted) { reason_ = "UP ignored: no fresh heartbeat"; return ModeAction::None; }
        if (selected_ == DriveMode::Manual && !in.manualNeutral) {
          reason_ = "UP ignored: hold the handheld level (neutral) to start MANUAL";
          return ModeAction::None;
        }
        reason_ = selected_ == DriveMode::Manual ? "UP: start MANUAL" : "UP: start AUTO";
        return selected_ == DriveMode::Manual ? ModeAction::StartManual : ModeAction::StartAuto;
      case Gesture::None:
        break;
    }
    return ModeAction::None;
  }
  DriveMode selected() const { return selected_; }
  // Human-readable outcome of the last gesture, empty when there was none.
  const char* reason() const { return reason_; }

 private:
  DriveMode selected_ = DriveMode::Idle;  // nothing selected after power-up
  const char* reason_ = "";
};

}  // namespace control
}  // namespace mech368
#endif
