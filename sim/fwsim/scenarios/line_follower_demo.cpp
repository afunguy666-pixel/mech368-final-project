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
