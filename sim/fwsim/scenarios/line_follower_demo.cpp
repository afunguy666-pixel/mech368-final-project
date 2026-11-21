// Scenarios for sim/examples/line_follower (demo of motors, encoders, line
// sensors, ultrasonic, gestures, IMU and battery in the simulator).
#include "Simulation.h"
#include <Arduino_APDS9960.h>
#include <cmath>
#include <cstdio>

using namespace fwsim;

namespace {
void demo(Simulation& sim, const char* world) {
  sim.loadWorld(world);
  sim.loadVehicle("sim/vehicles/example.vehicle");
  sim.useSketch(Robot, "line_follower");
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
}  // namespace

FWSIM_SCENARIO(line_follow, "Demo: start by gesture, follow the oval for one lap, stop by gesture") {
  demo(sim, "sim/worlds/oval.world");
  sim.gesture(1000, GESTURE_UP);
  sim.gesture(27000, GESTURE_DOWN);
  sim.runUntil(900);
  const Pose2 start = sim.vehicle().pose;
  FWSIM_CHECK(sim, std::hypot(start.x - sim.world().start.x, start.y - sim.world().start.y) < 1e-6,
              "robot does not move before the UP gesture");
  sim.runUntil(27000);
  const double lap = sim.world().lineLength();
  char buf[128];
  std::snprintf(buf, sizeof buf, "travelled %.2f m of a %.2f m lap; max line offset %.1f mm",
                sim.vehicle().travelled, lap, sim.maxLineOffset(3000, 27000) * 1000);
  sim.log("check", buf);
  FWSIM_CHECK(sim, sim.vehicle().travelled >= lap, "completes at least one lap in 26 s");
  FWSIM_CHECK(sim, sim.maxLineOffset(3000, 27000) < 0.015, "sensor bar stays within 15 mm of the line");
  FWSIM_CHECK(sim, sim.vehicle().collisions == 0, "no collisions");
  sim.runUntil(29000);
  FWSIM_CHECK(sim, speedAt(sim, 28900) < 0.01, "robot stopped after the DOWN gesture");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "gesture DOWN"), "sketch logged the DOWN gesture");
}

FWSIM_SCENARIO(obstacle_stop, "Demo: stop before a block on the line, resume when it is removed") {
  demo(sim, "sim/worlds/oval_obstacle.world");
  sim.gesture(500, GESTURE_UP);
  sim.at(25000, [](Simulation& s) { s.world().boxes.clear(); }, "block removed from the course");
  sim.runUntil(24000);
  FWSIM_CHECK(sim, sim.vehicle().collisions == 0, "robot never touches the block");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "obstacle at"), "sketch detected the obstacle");
  FWSIM_CHECK(sim, speedAt(sim, 23900) < 0.01, "robot is standing still in front of the block");
  const double gap = sim.vehicle().distanceReading(0);
  char buf[96];
  std::snprintf(buf, sizeof buf, "ultrasonic distance while stopped: %.1f cm", gap * 100);
  sim.log("check", buf);
  FWSIM_CHECK(sim, gap > 0.05 && gap < 0.25, "stops between 5 and 25 cm from the block");
  sim.runUntil(28000);
  FWSIM_CHECK(sim, speedAt(sim, 27900) > 0.1, "robot resumes after the block is removed");
}

FWSIM_SCENARIO(low_battery, "Demo: red LED turns on when the battery sags below 6.8 V") {
  demo(sim, "sim/worlds/oval.world");
  sim.setBattery(3000, 6.5);
  sim.runUntil(5000);
  FWSIM_CHECK(sim, sim.levelAt(Robot, LEDR, 2900) == HIGH, "red LED off (active low) with a full battery");
  FWSIM_CHECK(sim, sim.levelAt(Robot, LEDR, 3600) == LOW, "red LED on within 600 ms of the voltage drop");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "LOW BATTERY"), "sketch reports the low battery");
}

// ---------------------------------------------------- handout field map ---
namespace {
double closestApproach(const Simulation& sim, Vec2 p) {
  double best = 1e9;
  for (const auto& s : sim.samples()) best = std::min(best, std::hypot(s.pose.x - p.x, s.pose.y - p.y));
  return best;
}
}  // namespace

FWSIM_SCENARIO(course_first_cone, "Demo on the handout field: stop before the cone on the first leg") {
  demo(sim, "sim/worlds/course.world");
  sim.gesture(500, GESTURE_UP);
  sim.runUntil(12000);
  FWSIM_CHECK(sim, sim.vehicle().collisions == 0, "robot never touches a cone");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "obstacle at"), "sketch detected the cone");
  FWSIM_CHECK(sim, speedAt(sim, 11900) < 0.01, "robot is standing still in front of the cone");
  FWSIM_CHECK(sim, sim.vehicle().pose.y > 0.3, "robot drove up the first leg before stopping");
}

FWSIM_SCENARIO(course_serpentine, "Demo on the handout field without cones: follow all 90-degree turns to the end") {
  demo(sim, "sim/worlds/course.world");
  // Pure line-following test. With the border kept, the demo's "stop below
  // 20 cm" rule sees the wall ~20 cm beyond each bottom joint and stops.
  sim.at(0, [](Simulation& s) {
    s.world().circles.clear();
    s.world().walls.clear();
  }, "cones and border walls removed: pure line-following run");
  sim.gesture(500, GESTURE_UP);
  sim.runUntil(60000);
  const Vec2 end = sim.world().lines.at(0).pts.back();
  const double approach = closestApproach(sim, end);
  double reachedMs = -1;
  for (const auto& s : sim.samples())
    if (std::hypot(s.pose.x - end.x, s.pose.y - end.y) < 0.15) { reachedMs = s.t / 1000.0; break; }
  char buf[160];
  std::snprintf(buf, sizeof buf, "track length %.2f m; closest approach to the end %.1f cm; reached at %.1f s; "
                "max line offset %.1f mm", sim.world().lineLength(), approach * 100, reachedMs / 1000,
                sim.maxLineOffset(1000, reachedMs > 0 ? reachedMs : 60000) * 1000);
  sim.log("check", buf);
  FWSIM_CHECK(sim, reachedMs > 0, "robot reaches the end of the track (within 15 cm)");
  FWSIM_CHECK(sim, reachedMs < 0 || sim.maxLineOffset(1000, reachedMs) < 0.06,
              "sensor bar stays within 6 cm of the line centre through all turns");
  FWSIM_CHECK(sim, sim.vehicle().collisions == 0, "no collisions");
}
