#ifndef MECH368_DRIVE_CALIBRATION_H
#define MECH368_DRIVE_CALIBRATION_H
#include <DriveMath.h>

namespace mech368 {
namespace calibration {
// CAD / simulator defaults. Replace each wheel independently after measurement.
struct Wheel {
  int forwardSign;             // raw decoder sign when rolled forward
  float countsPerRevolution;   // wheel revolutions, channel A CHANGE (x2)
  float rolloutMetres;         // loaded travel per wheel revolution
  float travel(uint32_t count, uint32_t previous) const {
    return static_cast<int32_t>(count - previous) * forwardSign *
           (rolloutMetres / countsPerRevolution);
  }
};
#ifdef MECH368_CALIBRATION_HEADER
#include MECH368_CALIBRATION_HEADER
#else
constexpr Wheel left = {1, 600.0f, 0.065f * line::kPi};
constexpr Wheel right = {1, 600.0f, 0.065f * line::kPi};
constexpr float trackMetres = 0.092f;
#endif
constexpr float batteryDivider = 3.0f;  // Vpack / VA7, provisional
} // namespace calibration
} // namespace mech368
#endif
