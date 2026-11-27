// Low-battery indicator on the robot sketch (A7 divider, D1 LOW BAT LED,
// power LED double dip). The plant has no load sag, so the motor-start sag
// is scripted as short voltage steps.
#include "Simulation.h"
#include <Arduino_APDS9960.h>
#include <cstdio>

using namespace fwsim;

namespace {
int transitions(Simulation& sim, int pin, double fromMs, double toMs) {
  int n = 0;
  for (const auto& e : sim.board(Robot).pinLog)
    if (e.pin == pin && e.t >= fromMs * 1000 && e.t <= toMs * 1000) ++n;
  return n;
}

double firstChange(Simulation& sim, int pin, int value, double fromMs) {
  for (const auto& e : sim.board(Robot).pinLog)
    if (e.pin == pin && e.value == value && e.t >= fromMs * 1000) return e.t / 1000.0;
  return -1;
}
}  // namespace

FWSIM_SCENARIO(battery_low_indicator, "Low-battery LED: filtered, held and with hysteresis; separate from the BLE light") {
  sim.loadWorld("sim/worlds/course.world");
  sim.loadVehicle("sim/vehicles/ir_tracker.vehicle");  // 7.2 V pack, 3:1 divider (assumed)
  sim.useSketch(Robot, "robot");
  sim.useSketch(Remote, "remote", 300);
  // Motor-start style sags: 0.8 s at 5.6 V, twice.
  sim.setBattery(4000, 5.6);
  sim.setBattery(4800, 7.2);
  sim.setBattery(6000, 5.6);
  sim.setBattery(6800, 7.2);
  sim.setBattery(9000, 6.2);   // sustained: below the 6.4 V threshold
  sim.setBattery(18000, 6.6);  // inside the hysteresis band
  sim.setBattery(30000, 7.0);  // above the 6.8 V clear threshold
  sim.setBattery(44000, 0.0);  // pack switched off, board on USB
  sim.runUntil(47000);

  const double on = firstChange(sim, D1, HIGH, 0);
  const double off = firstChange(sim, D1, LOW, 9000);
  char buf[160];
  std::snprintf(buf, sizeof buf, "LOW BAT on at %.0f ms (drop at 9000 ms), off at %.0f ms (recovery at 30000 ms)", on, off);
  sim.log("check", buf);
  FWSIM_CHECK(sim, sim.steady(Robot, D1, LOW, 100, 9000), "two 0.8 s sags do not light LOW BAT");
  FWSIM_CHECK(sim, on > 14000 && on < 17000, "sustained low voltage lights LOW BAT after filter and 5 s hold");
  FWSIM_CHECK(sim, sim.steady(Robot, D1, HIGH, on + 1, 30000), "stays lit inside the hysteresis band");
  FWSIM_CHECK(sim, off > 40000 && off < 42500, "clears only after 10 s above 6.8 V");
  FWSIM_CHECK(sim, sim.steady(Robot, D1, LOW, off + 1, 47000), "no pack (USB only) is not reported as LOW");
  FWSIM_CHECK(sim, sim.steady(Robot, LED_PWR, HIGH, 100, on - 1) && transitions(sim, LED_PWR, on, 29000) >= 20,
              "power LED double-dips only while LOW");
  FWSIM_CHECK(sim, sim.steady(Robot, LED_BUILTIN, HIGH, 20000, 29000),
              "BLE light stays solid (link healthy) while LOW BAT is lit");
  FWSIM_CHECK(sim, sim.serialContains(Remote, "@"), "telemetry still forwarded");
}
