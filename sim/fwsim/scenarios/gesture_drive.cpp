// Gesture steering (handheld tilt -> MANUAL) on the paired robot and remote
// sketches: explicit selection and start, direction, mode switch, link loss,
// E-stop and IMU failure. Open floor, so the course does not interfere.
#include "Simulation.h"
#include <Arduino_APDS9960.h>
#include <cmath>
#include <cstdio>

using namespace fwsim;
namespace wire = mech368::ble;

namespace {
void openFloor(Simulation& sim) {
  sim.loadWorld("sim/worlds/course.world");
  sim.loadVehicle("sim/vehicles/ir_tracker.vehicle");
  auto& w = sim.world();
  w.width = w.height = 20;
  w.start = {10, 10, 0};
  w.circles.clear();
  w.lines.clear();
  sim.useSketch(Robot, "robot");
  sim.useSketch(Remote, "remote", 300);
  sim.tilt(0, 0, 0);  // hand still and level while the remote zeroes
}

// Select MANUAL and start it with the handheld level.
void startManual(Simulation& sim, double selectMs = 2000, double upMs = 2600) {
  sim.gesture(selectMs, GESTURE_LEFT);
  sim.gesture(upMs, GESTURE_UP);
}

// Hold a hand pose from fromMs to toMs with 200 ms ramps either side.
void hold(Simulation& sim, double fromMs, double toMs, double roll, double pitch) {
  sim.tilt(fromMs, 0, 0);
  sim.tilt(fromMs + 200, roll, pitch);
  sim.tilt(toMs, roll, pitch);
  sim.tilt(toMs + 200, 0, 0);
}

const Sample* at(const Simulation& sim, double ms) {
  for (const auto& s : sim.samples())
    if (s.t >= ms * 1000) return &s;
  return nullptr;
}

double speedAt(const Simulation& sim, double ms) {
  const auto& s = sim.samples();
  for (size_t i = 1; i < s.size(); ++i)
    if (s[i].t >= ms * 1000) {
      const double dt = (s[i].t - s[i - 1].t) / 1e6;
      return std::hypot(s[i].pose.x - s[i - 1].pose.x, s[i].pose.y - s[i - 1].pose.y) / dt;
    }
  return 0;
}

double travelled(const Simulation& sim, double fromMs, double toMs) {
  const Sample *a = at(sim, fromMs), *b = at(sim, toMs);
  return a && b ? std::hypot(b->pose.x - a->pose.x, b->pose.y - a->pose.y) : 0;
}

bool rgb(const Simulation& sim, double ms, bool r, bool g, bool b) {
  return (sim.levelAt(Robot, LEDR, ms) == LOW) == r && (sim.levelAt(Robot, LEDG, ms) == LOW) == g &&
         (sim.levelAt(Robot, LEDB, ms) == LOW) == b;
}

// Largest |linear| / |angular| in delivered Motion frames sent in [from, to] ms.
void commandRange(const Simulation& sim, double fromMs, double toMs, int& maxLinear, int& maxAngular) {
  maxLinear = maxAngular = 0;
  for (const auto& f : sim.frames()) {
    if (!f.fromRemote || !f.decoded || f.frame.type != wire::Type::Motion) continue;
    if (f.sentUs < fromMs * 1000 || f.sentUs > toMs * 1000) continue;
    maxLinear = std::max(maxLinear, std::abs(static_cast<int>(f.frame.linear)));
    maxAngular = std::max(maxAngular, std::abs(static_cast<int>(f.frame.angular)));
  }
}

// D0 toggles on each accepted packet that permits motion (bench trace build).
void checkDriveDeadline(Simulation& sim, double faultMs, double lastPossibleMs) {
  double accepted = -1, disabled = -1;
  for (const auto& pin : sim.board(Robot).pinLog) {
    if (pin.pin == D0 && pin.t <= lastPossibleMs * 1000) accepted = pin.t / 1000.0;
    if (pin.pin == D8 && !pin.value && pin.t >= faultMs * 1000 && disabled < 0) disabled = pin.t / 1000.0;
  }
  char timing[150];
  std::snprintf(timing, sizeof timing, "last accepted motion %.3f ms, D8 low %.3f ms, elapsed %.3f ms",
                accepted, disabled, disabled - accepted);
  sim.log("check", timing);
  FWSIM_CHECK(sim, accepted > faultMs - 150 && sim.levelAt(Robot, D8, accepted) == HIGH &&
              disabled >= accepted && disabled - accepted <= 500,
              "MANUAL: last accepted packet to nSLEEP low within 500 ms");
}
}  // namespace

FWSIM_SCENARIO(manual_tilt_drive, "MANUAL: explicit select and start, tilt directions, switch to AUTO stops") {
  openFloor(sim);
  sim.gesture(1500, GESTURE_UP);          // nothing selected yet
  sim.gesture(2000, GESTURE_LEFT);        // select MANUAL
  hold(sim, 2300, 3300, 0, 20);           // operator already tipping forward...
  sim.gesture(3000, GESTURE_UP);          // ...so this UP is refused
  sim.gesture(4000, GESTURE_UP);          // level again: MANUAL starts
  hold(sim, 5000, 7000, 0, 20);           // front edge down: forward
  hold(sim, 8000, 8800, 20, 0);           // right edge down: turn right
  hold(sim, 9500, 10500, 0, -20);         // back edge down: reverse
  sim.gesture(11500, GESTURE_RIGHT);      // select AUTO while driving: stop only
  hold(sim, 12000, 13000, 0, 20);         // tilt has no effect now
  sim.runUntil(14000);

  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 50, 3950), "UP without selection, and UP while tilted, leave the bridge off");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "select MANUAL (LEFT) or AUTO (RIGHT) first") &&
              sim.serialContains(Robot, "hold the handheld level"), "both refusals are logged on the robot");
  FWSIM_CHECK(sim, rgb(sim, 2500, true, false, true), "magenta: MANUAL selected, idle");
  FWSIM_CHECK(sim, sim.levelAt(Robot, D8, 4200) == HIGH && speedAt(sim, 4800) < 0.005,
              "MANUAL starts with the handheld level and the robot stays still");
  FWSIM_CHECK(sim, rgb(sim, 4800, true, true, false), "yellow: MANUAL driving");
  const Sample *s0 = at(sim, 5000), *s1 = at(sim, 7100);
  const double dx = s1->pose.x - s0->pose.x, dy = s1->pose.y - s0->pose.y;
  char buf[160];
  std::snprintf(buf, sizeof buf, "forward tilt moved %.3f m along x, %.3f m along y; speed %.3f m/s",
                dx, dy, speedAt(sim, 6800));
  sim.log("check", buf);
  FWSIM_CHECK(sim, dx > 0.25 && std::fabs(dy) < 0.05, "front edge down drives forward");
  FWSIM_CHECK(sim, speedAt(sim, 6800) < 0.31, "speed within the configured limits");
  FWSIM_CHECK(sim, speedAt(sim, 7800) < 0.005 && sim.levelAt(Robot, D8, 7800) == HIGH,
              "level handheld: wheels stop, MANUAL stays selected");
  const double turned = at(sim, 9300)->pose.yaw - at(sim, 7900)->pose.yaw;
  std::snprintf(buf, sizeof buf, "right-edge-down turned %.2f rad", turned);
  sim.log("check", buf);
  FWSIM_CHECK(sim, turned < -0.4 && travelled(sim, 7900, 9300) < 0.05, "right edge down turns right on the spot");
  const Sample *r0 = at(sim, 9500), *r1 = at(sim, 11000);
  const double back = std::cos(r0->pose.yaw) * (r1->pose.x - r0->pose.x) +
                      std::sin(r0->pose.yaw) * (r1->pose.y - r0->pose.y);
  FWSIM_CHECK(sim, back < -0.15, "back edge down reverses");
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 11600, 14000) && travelled(sim, 12000, 14000) < 0.005,
              "RIGHT while in MANUAL stops; tilt alone cannot drive in AUTO-selected idle");
  FWSIM_CHECK(sim, rgb(sim, 13500, false, false, true), "blue: AUTO selected, idle");
  int lin = 0, ang = 0;
  commandRange(sim, 5500, 7000, lin, ang);
  FWSIM_CHECK(sim, lin > 100 && lin <= 250, "remote sends forward speed in the Motion frame");
}

FWSIM_SCENARIO(manual_link_loss, "MANUAL: heartbeat loss and reconnect while tilted never restart") {
  openFloor(sim);
  startManual(sim);
  hold(sim, 3500, 20000, 0, 20);          // keep tipping forward throughout
  sim.block(true, 6000, 7000);            // remote -> robot packets lost
  sim.linkLoss(8000, 9500);               // full disconnect, then reconnect
  sim.gesture(11000, GESTURE_UP);         // MANUAL still selected, but tilted
  sim.runUntil(12500);
  FWSIM_CHECK(sim, speedAt(sim, 5800) > 0.1, "driving forward before the loss");
  checkDriveDeadline(sim, 6000, 6700);
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 6500, 12500), "bridge stays off through recovery, reconnect and a tilted UP");
  FWSIM_CHECK(sim, speedAt(sim, 12000) < 0.005, "robot at rest after reconnecting with the handheld tipped");
  const auto* status = sim.statusSentBefore(10800);
  FWSIM_CHECK(sim, status && (status->frame.linear & wire::MotionPermitted) &&
              (status->frame.linear & wire::OutputsDisabled), "fresh heartbeats alone leave outputs disabled");
}

FWSIM_SCENARIO(manual_estop, "MANUAL: remote E-stop wins, local unlock, separate level UP restarts") {
  openFloor(sim);
  startManual(sim);
  hold(sim, 3500, 9000, 0, 20);
  sim.serialInput(Remote, 5000, "e");
  sim.gesture(6000, GESTURE_LEFT);        // clears the latch only
  sim.gesture(7000, GESTURE_UP);          // tilted: refused
  sim.gesture(10000, GESTURE_UP);         // level: MANUAL restarts
  hold(sim, 10500, 11500, 0, 20);
  sim.setInput(Remote, 12000, D2, LOW);   // hardware stop button
  sim.setInput(Remote, 12200, D2, HIGH);
  sim.runUntil(13500);
  FWSIM_CHECK(sim, speedAt(sim, 4900) > 0.1, "driving before the stop");
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 5200, 9950), "E-stop cuts MANUAL; unlock and tilted UP keep it off");
  const auto* latched = sim.statusSentBefore(5900);
  const auto* cleared = sim.statusSentBefore(6800);
  FWSIM_CHECK(sim, latched && (latched->frame.linear & wire::EstopLatched), "stop latched");
  FWSIM_CHECK(sim, cleared && !(cleared->frame.linear & wire::EstopLatched), "local gesture cleared the latch");
  FWSIM_CHECK(sim, sim.levelAt(Robot, D8, 10200) == HIGH && speedAt(sim, 11300) > 0.1, "a separate level UP restarts MANUAL");
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 12300, 13500), "D2 button stops MANUAL");
}

FWSIM_SCENARIO(manual_imu_stale, "MANUAL: frozen handheld IMU sends zero, returns only through neutral") {
  openFloor(sim);
  startManual(sim);
  hold(sim, 3500, 9500, 0, 20);
  sim.at(5000, [](Simulation& s) { s.board(Remote).imuFrozen = true; }, "handheld IMU frozen");
  sim.at(6500, [](Simulation& s) { s.board(Remote).imuFrozen = false; }, "handheld IMU back, still tipped");
  hold(sim, 10500, 11500, 0, 20);
  sim.runUntil(12500);
  int lin = 0, ang = 0;
  commandRange(sim, 5110, 9600, lin, ang);
  FWSIM_CHECK(sim, speedAt(sim, 4900) > 0.1, "driving before the fault");
  FWSIM_CHECK(sim, lin == 0 && ang == 0, "remote sends exactly zero from 100 ms after the last sample until neutral");
  FWSIM_CHECK(sim, speedAt(sim, 5600) < 0.01 && speedAt(sim, 9000) < 0.005, "robot stops and does not lurch when data returns tilted");
  FWSIM_CHECK(sim, speedAt(sim, 11300) > 0.1, "after passing neutral, tilt drives again");
  FWSIM_CHECK(sim, sim.levelAt(Remote, LEDR, 8000) == LOW && sim.levelAt(Remote, LEDB, 8000) == LOW,
              "handheld magenta: waiting for neutral");
}

FWSIM_SCENARIO(manual_imu_absent, "MANUAL: handheld without a working IMU only sends zero") {
  openFloor(sim);
  sim.at(0, [](Simulation& s) { s.board(Remote).imuBeginFails = true; }, "handheld IMU.begin fails");
  startManual(sim);
  hold(sim, 3500, 6000, 25, 20);
  sim.runUntil(7000);
  int lin = 0, ang = 0;
  commandRange(sim, 0, 7000, lin, ang);
  FWSIM_CHECK(sim, lin == 0 && ang == 0, "all Motion frames carry zero");
  FWSIM_CHECK(sim, travelled(sim, 2000, 7000) < 0.005, "robot does not move");
  FWSIM_CHECK(sim, sim.levelAt(Remote, LEDR, 5000) == LOW && sim.levelAt(Remote, LEDG, 5000) == HIGH,
              "handheld red: no IMU");
}
