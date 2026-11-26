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

// Same plant for calibration and the held-out course. The estimator never
// receives these plant constants. The bench CSV comes from USB counts and a
// wheel index / chassis ruler / protractor in the model's world frame.
void loadedPlant(Simulation& sim) {
  auto& v = sim.vehicleConfig();
  v.leftScale = 0.955;
  v.rightScale = 0.975;
  v.trackWidth = 0.095;
  v.encLeft.ppr = 304;
  v.encRight.ppr = 298;
  v.gyroBiasDps = 0.65;
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

void checkDriveDeadline(Simulation& sim, double faultMs, double lastPossibleMs) {
  double accepted = -1, disabled = -1;
  for (const auto& pin : sim.board(Robot).pinLog) {
    if (pin.pin == D0 && pin.t <= lastPossibleMs * 1000) accepted = pin.t / 1000.0;
    if (pin.pin == D8 && !pin.value && pin.t >= faultMs * 1000 && disabled < 0)
      disabled = pin.t / 1000.0;
  }
  char timing[150];
  std::snprintf(timing, sizeof timing, "last accepted motion %.3f ms, D8 low %.3f ms, elapsed %.3f ms",
                accepted, disabled, disabled - accepted);
  sim.log("check", timing);
  FWSIM_CHECK(sim, accepted > faultMs - 150 && sim.levelAt(Robot, D8, accepted) == HIGH &&
              disabled >= accepted && disabled - accepted <= 500, "accepted motion to nSLEEP within 500 ms");
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
    for (int bearing : {-20, -10, 0, 10, 20}) {
      // Probe the centre ray, both edges of the 20-degree field, and beyond each edge.
      const double forward = 0.07825 + 0.060 * std::cos(bearing * rad);
      const double lateral = side * (0.020 + 0.060 * std::sin(bearing * rad));
      bench.circles = {{{bench.start.x - lateral, bench.start.y + forward}, 0.0005, "target"}};
      int left = -1, right = -1;
      const bool pins = car.digitalInput(sim.board(Robot), D12, left) &&
                        car.digitalInput(sim.board(Robot), D11, right);
      const bool inBeam = std::abs(bearing) <= 10;
      const bool leftHit = side > 0 && inBeam, rightHit = side < 0 && inBeam;
      const std::string label = std::string(side > 0 ? "Left" : "Right") + " mount, " +
          (bearing == 0 ? "straight ahead" :
           std::to_string(std::abs(bearing)) + (bearing > 0 ? " deg outboard" : " deg inboard"));
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

FWSIM_SCENARIO(telemetry_live_course, "Paired live lap, with a lossy USB interval for comparison") {
  pairedTracker(sim);
  sim.at(11500, [](Simulation& s) { s.board(Remote).usbWriteLimit = 7; }, "slow USB host");
  sim.at(12100, [](Simulation& s) { s.board(Remote).usbWriteLimit = 64; }, "USB host caught up");
  runCourse(sim, true);
  FWSIM_CHECK(sim, sim.serialContains(Remote, "@", 12500), "remote resumes forwarding after truncated lines");
}

FWSIM_SCENARIO(telemetry_loaded_course, "Held-out three-cone course with loaded tyres and different encoder ratios") {
  pairedTracker(sim);
  loadedPlant(sim);
  runCourse(sim, true);
}

FWSIM_SCENARIO(telemetry_bench, "Passive wheel index, loaded ruler and protractor with the bridge asleep") {
  sim.loadWorld("sim/worlds/course.world");
  sim.loadVehicle("sim/vehicles/ir_tracker.vehicle");
  loadedPlant(sim);
  auto& w = sim.world();
  w.width = w.height = 20;
  w.start = {10, 10, 0};
  w.circles.clear();
  sim.useSketch(Robot, "robot");
  sim.useSketch(Remote, "remote", 300);
  sim.at(0, [](Simulation& s) { s.vehicle().handRolled = true; });
  auto roll = [&](double ms, double left, double right) {
    sim.at(ms, [left, right](Simulation& s) {
      s.vehicle().handOmegaL = left;
      s.vehicle().handOmegaR = right;
    });
  };
  const auto& v = sim.vehicleConfig();
  const double l = v.wheelRadius * v.leftScale, r = v.wheelRadius * v.rightScale;
  const double turn = 2 * std::acos(-1.0);
  roll(3000, turn, 0); roll(5000, 0, 0);       // two marked left wheel turns
  roll(7500, 0, turn); roll(9500, 0, 0);       // two marked right wheel turns
  roll(12000, 0.20/l, 0.20/r); roll(16000, 0, 0);
  roll(18500, -0.05/l, 0.05/r); roll(22500, 0, 0);
  roll(25000, 0.05/l, -0.05/r); roll(29000, 0, 0);
  sim.runUntil(31500);
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 100, 31500), "all bench measurements passive, motor bridge asleep");
  FWSIM_CHECK(sim, sim.serialContains(Remote, "@", 30000), "includes final encoder plateau on handheld USB");
}

FWSIM_SCENARIO(telemetry_pressure, "USB unplug, full endpoint and reserved radio credit under motion and E-stop") {
  pairedTracker(sim);
  sim.at(6500, [](Simulation& s) { s.board(Remote).usbAttached = false; }, "unplug USB, remote separately powered");
  sim.at(7500, [](Simulation& s) { s.board(Remote).usbAttached = true; s.board(Remote).usbWriteLimit = 0; }, "USB attached, host does not read");
  sim.at(8200, [](Simulation& s) { s.board(Robot).aclCredits = 1; }, "only the control credit remains");
  sim.setInput(Remote, 8800, D2, LOW);
  sim.setInput(Remote, 9000, D2, HIGH);
  sim.linkLoss(9500, 11000);
  sim.at(11200, [](Simulation& s) { s.board(Remote).usbWriteLimit = 64; s.board(Robot).aclCredits = 4; });
  sim.runUntil(16000);
  int linesBefore = 0, during = 0, after = 0;
  for (const auto& line : sim.board(Remote).serialOut) {
    if (line.text.empty() || line.text.front() != '@') continue;
    if (line.t < 6500000) ++linesBefore;
    else if (line.t < 11200000) ++during;
    else ++after;
  }
  FWSIM_CHECK(sim, linesBefore > 20 && !during && after > 10, "lossy USB drops telemetry and resumes");
  int telemetryWrites = 0, statusWrites = 0;
  for (const auto& frame : sim.frames()) {
    if (frame.sentUs < 8210000 || frame.sentUs > 8750000) continue;
    if (frame.telemetryKind) ++telemetryWrites;
    if (frame.decoded && frame.frame.type == wire::Type::Status) ++statusWrites;
  }
  FWSIM_CHECK(sim, !telemetryWrites && statusWrites >= 9, "the last ACL credit is used only for control");
  FWSIM_CHECK(sim, sim.steady(Robot, D8, HIGH, 6600, 8750) &&
              sim.steady(Remote, LED_BUILTIN, HIGH, 6600, 8750), "drive and heartbeat survive unplug and congestion");
  const auto* last = sim.statusSentBefore(15700);
  FWSIM_CHECK(sim, last && (last->frame.linear & wire::EstopLatched) &&
              sim.steady(Robot, D8, LOW, 8970, 16000), "remote stop cuts drive and stays latched after reconnect");
}

FWSIM_SCENARIO(telemetry_deadline, "No controller credits during calibration, measure accepted heartbeat to nSLEEP") {
  pairedTracker(sim);
  sim.at(4050, [](Simulation& s) { s.board(Robot).aclCredits = 0; }, "controller TX full");
  sim.at(6500, [](Simulation& s) { s.board(Robot).aclCredits = 4; }, "credits restored");
  sim.runUntil(10000);
  checkDriveDeadline(sim, 4050, 4700);
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 4600, 10000), "no spontaneous start as heartbeats resume");
}

FWSIM_SCENARIO(telemetry_usb_power_loss, "Unplug the only supply to the handheld on a moving run") {
  pairedTracker(sim);
  sim.powerOff(Remote, 8000);
  sim.runUntil(12000);
  checkDriveDeadline(sim, 8000, 8600);
  FWSIM_CHECK(sim, sim.steady(Robot, D8, LOW, 8500, 12000), "unpowered remote cannot keep motors enabled");
}

FWSIM_SCENARIO(robot_track_watchdog, "Paired firmware: heartbeat loss drops nSLEEP and cancels navigation") {
  pairedTracker(sim);
  sim.block(true, 8000, 10000);
  sim.gesture(9000, GESTURE_UP);  // discard a start request while the link is stale
  sim.gesture(13500, GESTURE_UP);
  sim.setInput(Robot, 14100, A6, LOW);
  sim.setInput(Robot, 14300, A6, HIGH);
  sim.runUntil(14600);
  checkDriveDeadline(sim, 8000, 8900);
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
