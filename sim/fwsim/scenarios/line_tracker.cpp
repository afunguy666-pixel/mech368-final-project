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
namespace wire = mech368::ble;

namespace {
void tracker(Simulation& sim, const char* vehicle) {
  sim.loadWorld("sim/worlds/course.world");
  sim.loadVehicle(vehicle);
  sim.useSketch(Robot, "line_tracker");
  sim.gesture(1500, GESTURE_UP);  // 1.5 s still for the gyro bias first
}

void pairedTracker(Simulation& sim) {
  sim.loadWorld("sim/worlds/course.world");
  sim.loadVehicle("sim/vehicles/ir_tracker.vehicle");
  sim.useSketch(Robot, "robot");
  sim.useSketch(Remote, "remote", 300);
  sim.gesture(800, GESTURE_UP);  // still no valid heartbeat
  sim.gesture(3500, GESTURE_UP);
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

// Probe the signed mount angles independently of navigation. With the car facing
// world +y, a target to its left is at a smaller world x.
void checkObstacleMount(Simulation& sim) {
  World bench;
  const double rad = std::acos(-1.0) / 180.0;
  bench.start = {0.4, 0.4, 90 * rad};
  Vehicle car(sim.vehicleConfig(), bench);
  for (auto& sensor : car.cfg.distance) sensor.noise = 0;
  for (int side : {1, -1}) {
    for (int bearing : {10, 20, -10}) {
      // 1 mm diameter target, 60 mm from the CAD lens position. Test the
      // centre ray, the outer edge, and the mirror of the centre ray inboard.
      const double forward = 0.07825 + 0.060 * std::cos(bearing * rad);
      const double lateral = side * (0.020 + 0.060 * std::sin(bearing * rad));
      bench.circles = {{{bench.start.x - lateral, bench.start.y + forward}, 0.0005, "target"}};
      int left = -1, right = -1;
      const bool pins = car.digitalInput(sim.board(Robot), D12, left) &&
                        car.digitalInput(sim.board(Robot), D11, right);
      const bool leftHit = side > 0 && bearing > 0, rightHit = side < 0 && bearing > 0;
      const std::string label = std::string(side > 0 ? "Left" : "Right") + " mount, " +
          std::to_string(std::abs(bearing)) + (bearing > 0 ? " deg outboard" : " deg inboard");
      FWSIM_CHECK(sim, pins && left == (leftHit ? LOW : HIGH) && right == (rightHit ? LOW : HIGH), label);
    }
  }
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
    checkObstacleMount(sim);
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
  FWSIM_CHECK(sim, sim.levelAt(Robot, D8, 900) == LOW, "DRV8833 nSLEEP low before the start gesture");
  FWSIM_CHECK(sim, sim.levelAt(Robot, D8, 5000) == HIGH, "DRV8833 nSLEEP high while driving");
  FWSIM_CHECK(sim, sim.levelAt(Robot, D8, 9500) == LOW, "DRV8833 nSLEEP low again after the stop");
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

FWSIM_SCENARIO(robot_track_cones, "Paired firmware: follow the direct-wired three-cone course") {
  pairedTracker(sim);
  runCourse(sim, true);
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 50, 3450), "neither an early UP nor a heartbeat starts the driver");
  FWSIM_CHECK(sim, sim.levelAt(Robot, D8, 4200) == HIGH, "driver enabled during calibration");
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 70000, 75000), "driver asleep at the finish");
  const auto* running = sim.statusSentBefore(8000);
  const auto* finished = sim.statusSentBefore(74000);
  FWSIM_CHECK(sim, running && !(running->frame.linear & wire::OutputsDisabled), "BLE reports active outputs on track");
  FWSIM_CHECK(sim, finished && (finished->frame.linear & wire::OutputsDisabled), "BLE reports sleeping outputs at the finish");
}

FWSIM_SCENARIO(robot_track_watchdog, "Paired firmware: heartbeat loss drops nSLEEP and cancels navigation") {
  pairedTracker(sim);
  sim.block(true, 8000, 10000);
  sim.gesture(9000, GESTURE_UP);  // discard a start request while the link is stale
  sim.gesture(13500, GESTURE_UP);
  sim.setInput(Robot, 14100, A6, LOW);
  sim.setInput(Robot, 14300, A6, HIGH);
  sim.runUntil(14600);
  const double last = sim.lastCommandDelivered(8900);
  double disabled = -1;
  for (const auto& pin : sim.board(Robot).pinLog) {
    if (pin.pin == D8 && pin.value == LOW && pin.t >= last * 1000) {
      disabled = pin.t / 1000.0;
      break;
    }
  }
  char timing[120];
  std::snprintf(timing, sizeof timing, "last delivered heartbeat %.2f ms, nSLEEP low %.2f ms, elapsed %.2f ms",
                last, disabled, disabled - last);
  sim.log("check", timing);
  FWSIM_CHECK(sim, last > 7800 && sim.levelAt(Robot, D8, last) == HIGH, "driver was enabled at the last heartbeat");
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, last + 500, 13450), "nSLEEP low within 500 ms and through reconnection");
  FWSIM_CHECK(sim, speedAt(sim, 9600) < 0.01, "simulated wheels coast to rest");
  const auto* restored = sim.statusSentBefore(13300);
  FWSIM_CHECK(sim, restored && (restored->frame.linear & wire::MotionPermitted) &&
              (restored->frame.linear & wire::OutputsDisabled), "fresh heartbeats alone leave the driver off");
  FWSIM_CHECK(sim, sim.levelAt(Robot, D8, 13600) == HIGH, "a new local UP starts navigation after reconnection");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "DRV8833 FAULT: outputs disabled") &&
              sim.steady(Robot, D8, LOW, 14120, 14600), "a driver fault stops motion and fault release alone cannot restart it");
}

FWSIM_SCENARIO(robot_track_estop, "Paired firmware: E-stop during calibration, local recovery and a second stop on track") {
  pairedTracker(sim);
  sim.serialInput(Remote, 4100, "e");
  sim.linkLoss(5200, 6200);
  sim.gesture(10500, GESTURE_LEFT);  // any local gesture clears the latch
  sim.gesture(11500, GESTURE_UP);  // deliberate restart after a fresh heartbeat
  sim.serialInput(Remote, 18000, "e");
  sim.runUntil(19000);
  FWSIM_CHECK(sim, sim.levelAt(Robot, D8, 4050) == HIGH, "the calibration sweep was driving");
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 4300, 11450), "E-stop cuts calibration and the recovery gesture leaves nSLEEP low");
  const auto* latched = sim.statusSentBefore(10000);
  const auto* cleared = sim.statusSentBefore(11200);
  FWSIM_CHECK(sim, latched && (latched->frame.linear & wire::EstopLatched), "E-stop survives reconnection");
  FWSIM_CHECK(sim, cleared && !(cleared->frame.linear & wire::EstopLatched) &&
              (cleared->frame.linear & wire::MotionPermitted) && (cleared->frame.linear & wire::OutputsDisabled),
              "local LEFT clears only the latch; the remote resumes heartbeats without a reset");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "nav: follow", 11500) && sim.levelAt(Robot, D8, 17900) == HIGH,
              "a separate UP recalibrates and starts following");
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 18200, 19000), "the next remote E-stop cuts running navigation");
}

FWSIM_SCENARIO(robot_track_recovery_gestures, "Paired firmware: each APDS gesture can clear the E-stop latch") {
  pairedTracker(sim);
  const int gestures[] = {GESTURE_UP, GESTURE_DOWN, GESTURE_LEFT, GESTURE_RIGHT};
  double request = 4400;
  for (int gesture : gestures) {
    sim.serialInput(Remote, request, "e");
    sim.gesture(request + 400, gesture);
    sim.runUntil(request + 850);
    const auto* during = sim.statusSentBefore(request + 300);
    const auto* after = sim.statusSentBefore(request + 800);
    FWSIM_CHECK(sim, during && (during->frame.linear & wire::EstopLatched), "remote stop is latched before the gesture");
    FWSIM_CHECK(sim, after && !(after->frame.linear & wire::EstopLatched) &&
                (after->frame.linear & wire::MotionPermitted), "this local gesture releases the latch and fresh packets restore permission");
    FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, request + 200, request + 850), "recovery alone keeps the bridge asleep");
    request += 1000;
  }
}

FWSIM_SCENARIO(robot_track_stop_priority, "Paired firmware: fresh UP coincident with a delivered E-stop") {
  pairedTracker(sim);
  sim.serialInput(Remote, 8000, "e");
  sim.runUntil(7999);
  size_t seen = sim.frames().size();
  double arrival = -1;
  // The radio trace includes the in-flight packet's delivery time. Place the
  // gesture at that time under each seed, before either input reaches the car.
  for (int ms = 8000; ms < 8150 && arrival < 0; ++ms) {
    sim.runUntil(ms);
    for (; seen < sim.frames().size(); ++seen) {
      const auto& f = sim.frames()[seen];
      if (f.fromRemote && f.decoded && f.fate == RadioPacket::Delivered && f.frame.type == wire::Type::Stop)
        arrival = f.deliverUs / 1000.0;
    }
  }
  FWSIM_CHECK(sim, arrival > sim.nowMs(), "a real Stop frame is in flight");
  if (arrival <= sim.nowMs()) return;
  sim.gesture(arrival, GESTURE_UP);
  // Isolate the first stop: a retransmission must not hide an incorrect unlock.
  sim.block(true, arrival + 2, arrival + 400);
  sim.runUntil(arrival + 300);
  const auto* status = sim.statusSentBefore(sim.nowMs());
  FWSIM_CHECK(sim, status && (status->frame.linear & wire::EstopLatched), "the simultaneous UP cannot clear this stop");
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, arrival + 5, sim.nowMs()), "the driver stays asleep");
}
