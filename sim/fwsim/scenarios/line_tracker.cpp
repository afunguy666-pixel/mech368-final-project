// Scenarios for sim/examples/line_tracker (libraries/Mech368Line) on the
// handout field map (sim/worlds/course.world): serpentine track with six
// 90-degree corners and three cones, with the analog IR array and with the
// LM393 (digital) array.
#include "Simulation.h"
#include <Arduino_APDS9960.h>
#include <cmath>
#include <cstdio>
#include <string>

using namespace fwsim;

namespace {
void tracker(Simulation& sim, const char* vehicle) {
  sim.loadWorld("sim/worlds/course.world");
  sim.loadVehicle(vehicle);
  sim.useSketch(Robot, "line_tracker");
  sim.gesture(1500, GESTURE_UP);  // 1.5 s still for the gyro bias first
}

double closestApproach(const Simulation& sim, Vec2 p, double fromMs = 0) {
  double best = 1e9;
  for (const auto& s : sim.samples())
    if (s.t >= fromMs * 1000) best = std::min(best, std::hypot(s.pose.x - p.x, s.pose.y - p.y));
  return best;
}

double firstTimeNear(const Simulation& sim, Vec2 p, double radius) {
  for (const auto& s : sim.samples())
    if (std::hypot(s.pose.x - p.x, s.pose.y - p.y) < radius) return s.t / 1e6;  // s
  return -1;
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

// Largest sensor-bar offset from the line while the sketch reports "follow".
// Corner and detour manoeuvres leave the line on purpose and are excluded.
double followOffset(Simulation& sim, double endMs) {
  double worst = 0;
  bool following = false;
  size_t next = 0;
  const auto& ev = sim.board(Robot).serialOut;
  for (const auto& s : sim.samples()) {
    if (s.t > endMs * 1000) break;
    while (next < ev.size() && ev[next].t <= s.t) {
      const std::string& text = ev[next].text;
      if (text.rfind("nav: ", 0) == 0) following = text.rfind("nav: follow ", 0) == 0;
      ++next;
    }
    if (following) worst = std::max(worst, s.lineOffset);
  }
  return worst;
}

void runCourse(Simulation& sim, bool cones) {
  if (!cones) sim.at(0, [](Simulation& s) { s.world().circles.clear(); }, "cones removed");
  sim.runUntil(75000);
  const Vec2 end = sim.world().lines.at(0).pts.back();
  const double reached = firstTimeNear(sim, end, 0.10);
  char buf[200];
  std::snprintf(buf, sizeof buf, "track %.2f m; end reached at %.1f s; closest to end %.1f cm; "
                "max offset while following %.1f mm; collisions %d",
                sim.world().lineLength(), reached, closestApproach(sim, end) * 100,
                followOffset(sim, reached > 0 ? reached * 1000 : 75000) * 1000, sim.vehicle().collisions);
  sim.log("check", buf);
  FWSIM_CHECK(sim, sim.serialContains(Robot, "calibration min/max"), "line sensors calibrated by the sweep");
  FWSIM_CHECK(sim, !sim.serialContains(Robot, "FAILED"), "calibration accepted");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "nav stopped: finished"), "navigator stopped at the end of the track");
  if (!cones)
    FWSIM_CHECK(sim, sim.serialContains(Robot, "6 corners"), "all six 90-degree corners taken as corners");
  FWSIM_CHECK(sim, reached > 0, "robot reaches the end of the track (within 10 cm)");
  FWSIM_CHECK(sim, reached < 0 || reached < 60, "track finished within 60 s");
  FWSIM_CHECK(sim, followOffset(sim, reached > 0 ? reached * 1000 : 75000) < 0.03,
              "sensor bar within 3 cm of the line centre while following");
  FWSIM_CHECK(sim, sim.vehicle().collisions == 0, "no collisions with cones or the border");
  FWSIM_CHECK(sim, speedAt(sim, 74900) < 0.01, "robot stopped at the end");
  if (cones) {
    // The detour around the cone on the bottom joint meets the next leg and
    // pivots onto it, so that corner is taken by the rejoin, not as a corner.
    FWSIM_CHECK(sim, sim.serialContains(Robot, "3 detours"), "went around all three cones");
    FWSIM_CHECK(sim, !sim.serialContains(Robot, "nav: blocked") && !sim.serialContains(Robot, "nav: lost"),
                "no blocked or lost stops (no human intervention needed)");
  }
}
}  // namespace

FWSIM_SCENARIO(track_serpentine, "Line tracker (analog IR array): handout field without cones, start to end") {
  tracker(sim, "sim/vehicles/ir_tracker.vehicle");
  runCourse(sim, false);
}

FWSIM_SCENARIO(track_cones, "Line tracker (analog IR array): handout field with the three cones") {
  tracker(sim, "sim/vehicles/ir_tracker.vehicle");
  runCourse(sim, true);
}

FWSIM_SCENARIO(track_cones_lm393, "Line tracker (LM393 digital IR array): handout field with the three cones") {
  tracker(sim, "sim/vehicles/ir_tracker_lm393.vehicle");
  runCourse(sim, true);
}

FWSIM_SCENARIO(track_gesture_stop, "Line tracker: DOWN gesture stops the robot mid-track") {
  tracker(sim, "sim/vehicles/ir_tracker.vehicle");
  sim.gesture(8000, GESTURE_DOWN);
  sim.runUntil(10000);
  FWSIM_CHECK(sim, speedAt(sim, 4200) > 0.1, "robot was following the line before the gesture");
  FWSIM_CHECK(sim, speedAt(sim, 9500) < 0.01, "robot stopped after the DOWN gesture");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "gesture DOWN: stopping"), "sketch logged the stop");
}

FWSIM_SCENARIO(track_cones_on_line, "Line tracker: cones centred on the tape (handout Figure 5), detour side by sensor") {
  tracker(sim, "sim/vehicles/ir_tracker.vehicle");
  sim.at(0, [](Simulation& s) {
    auto& c = s.world().circles;
    c.at(0).c.x = 0.4425;  // left leg
    c.at(1).c.y = 0.1925;  // bottom joint: the right-hand detour meets the border
    c.at(2).c.x = 1.6075;  // right leg
  }, "cones moved onto the track centre line");
  runCourse(sim, true);
  FWSIM_CHECK(sim, sim.serialContains(Robot, "retrying on the"), "a blocked detour backed up and went round the other side");
}
