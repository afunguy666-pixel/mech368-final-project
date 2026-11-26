// Scenarios for the real firmware pair: firmware/robot + firmware/remote.
// They observe the firmware only from the outside, like bench equipment:
// LED pins, serial output, and the BLE frames on the (simulated) air.
#include "Simulation.h"
#include <cstdio>

using namespace fwsim;
namespace wire = mech368::ble;

namespace {
const double kWatchdogMs = mech368::MotionSafety::kTimeoutMs;

void boot(Simulation& sim, double remoteOnMs = 300, double robotOnMs = 0) {
  sim.loadWorld("sim/worlds/course.world");
  sim.loadVehicle("sim/vehicles/example.vehicle");
  sim.useSketch(Robot, "robot", robotOnMs);
  sim.useSketch(Remote, "remote", remoteOnMs);
}

bool flag(const FrameRecord* f, int bit) { return f && (f->frame.linear & bit) != 0; }

// Time (ms) the robot first reported motion permitted, or -1.
double firstPermitted(const Simulation& sim, double fromMs = 0) {
  for (const auto& f : sim.frames())
    if (!f.fromRemote && f.decoded && f.frame.type == wire::Type::Status &&
        f.sentUs >= fromMs * 1000 && (f.frame.linear & wire::MotionPermitted))
      return f.sentUs / 1000.0;
  return -1;
}

// No status may report "motion permitted" in [from, to].
bool neverPermitted(const Simulation& sim, double fromMs, double toMs) {
  for (const auto& f : sim.frames())
    if (!f.fromRemote && f.decoded && f.frame.type == wire::Type::Status &&
        f.sentUs >= fromMs * 1000 && f.sentUs <= toMs * 1000 && (f.frame.linear & wire::MotionPermitted))
      return false;
  return true;
}

// Safety invariant for every run: whenever the robot reports motion permitted,
// a motion command was delivered to it less than the watchdog period earlier.
void checkWatchdogInvariant(Simulation& sim) {
  int violations = 0;
  double worstMs = 0;
  for (const auto& status : sim.frames()) {
    if (status.fromRemote || !status.decoded || status.frame.type != wire::Type::Status ||
        !(status.frame.linear & wire::MotionPermitted))
      continue;
    double latest = -1;
    for (const auto& cmd : sim.frames())
      if (cmd.fromRemote && cmd.fate == RadioPacket::Delivered && cmd.decoded &&
          cmd.frame.type == wire::Type::Motion && cmd.deliverUs <= status.sentUs)
        latest = std::max(latest, cmd.deliverUs / 1000.0);
    const double age = latest < 0 ? 1e9 : status.sentUs / 1000.0 - latest;
    worstMs = std::max(worstMs, age);
    if (age > kWatchdogMs + 5) ++violations;  // 5 ms: robot polls after delivery
  }
  char buf[160];
  std::snprintf(buf, sizeof buf,
                "watchdog invariant: every 'permitted' status is < %.0f ms after a delivered "
                "motion command (worst %.1f ms)", kWatchdogMs, worstMs);
  FWSIM_CHECK(sim, violations == 0, buf);
}

void checkHealthy(Simulation& sim, double fromMs, double toMs, const char* when) {
  FWSIM_CHECK(sim, sim.steady(Remote, LED_BUILTIN, HIGH, fromMs, toMs),
              std::string("remote LED steady on (link healthy) ") + when);
  FWSIM_CHECK(sim, sim.steady(Robot, LED_BUILTIN, HIGH, fromMs, toMs),
              std::string("robot LED steady on (link + motion permission) ") + when);
  const FrameRecord* s = sim.statusSentBefore(toMs);
  FWSIM_CHECK(sim, flag(s, wire::AckValid) && flag(s, wire::MotionPermitted) &&
                       !flag(s, wire::EstopLatched) && flag(s, wire::OutputsDisabled),
              std::string("status flags: ACK + permitted, no E-stop, outputs disabled ") + when);
}
}  // namespace

FWSIM_SCENARIO(link_boot, "Robot boots, remote powers on 300 ms later; link becomes healthy") {
  boot(sim);
  sim.runUntil(5000);
  const double up = firstPermitted(sim);
  sim.log("check", "first status with motion permitted sent at " + std::to_string(up) + " ms");
  FWSIM_CHECK(sim, up > 0 && up < 3000, "link and heartbeat established within 3 s of power-up");
  FWSIM_CHECK(sim, sim.blinking(Remote, LED_BUILTIN, 300, 1200), "remote LED flashes while searching");
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, 0, 1200), "robot LED flashes before a link exists");
  checkHealthy(sim, 3500, 5000, "after boot");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(link_boot_robot_late, "Remote is on first; robot powers on 2.5 s later") {
  boot(sim, 0, 2500);
  sim.runUntil(8000);
  const double up = firstPermitted(sim);
  FWSIM_CHECK(sim, up > 2500 && up < 6000, "link established within 3.5 s after the robot powers on");
  FWSIM_CHECK(sim, sim.blinking(Remote, LED_BUILTIN, 0, 2500), "remote LED flashes while robot is off");
  checkHealthy(sim, 6500, 8000, "after late robot boot");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(command_silence, "Link stays connected but remote->robot commands stop for 2 s") {
  boot(sim);
  sim.block(true, 4000, 6000);
  sim.runUntil(10000);
  // Frames already in flight at 4 s still arrive, so look across the window.
  const double last = sim.lastCommandDelivered(6000);
  char buf[96];
  std::snprintf(buf, sizeof buf, "last command delivered at %.1f ms", last);
  sim.log("check", buf);
  FWSIM_CHECK(sim, neverPermitted(sim, last + kWatchdogMs + 5, 6000),
              "robot stops reporting 'motion permitted' within the 450 ms watchdog");
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, last + kWatchdogMs, 6000),
              "robot LED flashes while commands are missing");
  FWSIM_CHECK(sim, sim.blinking(Remote, LED_BUILTIN, 4000 + kWatchdogMs + 100, 6000),
              "remote LED flashes when its commands are no longer acknowledged");
  checkHealthy(sim, 8500, 10000, "after commands resume");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(status_silence, "Link stays connected but robot->remote status stops for 2 s") {
  boot(sim);
  sim.block(false, 4000, 6000);
  sim.runUntil(10000);
  FWSIM_CHECK(sim, sim.blinking(Remote, LED_BUILTIN, 4000 + kWatchdogMs + 100, 6000),
              "remote LED flashes when status/ACKs stop arriving");
  checkHealthy(sim, 8500, 10000, "after status resumes");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(link_loss, "Radio link lost (out of range) for 3 s, then restored") {
  boot(sim);
  sim.linkLoss(4000, 7000);
  sim.runUntil(12000);
  const double last = sim.lastCommandDelivered(7000);
  FWSIM_CHECK(sim, sim.levelAt(Robot, LED_BUILTIN, last + kWatchdogMs + 130) == LOW ||
                       sim.transitions(Robot, LED_BUILTIN, last, last + kWatchdogMs + 260) > 0,
              "robot LED leaves 'steady on' within watchdog + one flash phase");
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, 4500, 7000), "robot LED flashes during link loss");
  FWSIM_CHECK(sim, sim.blinking(Remote, LED_BUILTIN, 4600, 7000), "remote LED flashes during link loss");
  checkHealthy(sim, 10500, 12000, "after the link returns");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(remote_power_off, "Remote loses power while connected; robot must stop on its own") {
  boot(sim);
  sim.powerOff(Remote, 4000);
  sim.runUntil(8000);
  const double last = sim.lastCommandDelivered(8000);
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, last + kWatchdogMs, 8000),
              "robot LED flashes after the remote disappears");
  FWSIM_CHECK(sim, sim.hasEvent("closed connection") || sim.hasEvent("lost"),
              "robot side eventually observes the disconnect");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(estop_latch, "Remote E-stop ('e' on serial) latches through a link loss") {
  boot(sim);
  sim.serialInput(Remote, 4000, "e");
  sim.linkLoss(6000, 7000);
  sim.runUntil(11000);
  double latchedAt = -1;
  for (const auto& f : sim.frames())
    if (!f.fromRemote && f.decoded && f.frame.type == wire::Type::Status && f.sentUs >= 4000000 &&
        (f.frame.linear & wire::EstopLatched)) {
      latchedAt = f.sentUs / 1000.0;
      break;
    }
  FWSIM_CHECK(sim, latchedAt > 0 && latchedAt < 4200, "robot reports E-stop latched within 200 ms");
  FWSIM_CHECK(sim, neverPermitted(sim, latchedAt, 11000), "no motion permitted after the E-stop");
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, 4300, 6000), "robot LED flashes while latched");
  FWSIM_CHECK(sim, flag(sim.statusSentBefore(11000), wire::EstopLatched),
              "latch survives the link loss and reconnect");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(corrupt_commands, "Every remote->robot frame has one flipped bit for 1 s") {
  boot(sim);
  sim.at(4000, [](Simulation& s) { s.radio().corruptCentralToPeripheral = 1.0; },
         "radio: corrupt every remote -> robot frame");
  sim.at(5000, [](Simulation& s) { s.radio().corruptCentralToPeripheral = 0.0; },
         "radio: stop corrupting");
  sim.runUntil(9000);
  const double last = sim.lastCommandDelivered(5000);  // corrupted frames do not count
  FWSIM_CHECK(sim, neverPermitted(sim, last + kWatchdogMs + 5, 5000),
              "corrupted frames never extend the watchdog");
  checkHealthy(sim, 7500, 9000, "after corruption stops");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(lossy_link, "10% random packet loss in both directions for 10 s") {
  boot(sim);
  sim.at(2000, [](Simulation& s) {
    s.radio().lossCentralToPeripheral = 0.10;
    s.radio().lossPeripheralToCentral = 0.10;
  }, "radio: 10% loss both ways");
  sim.at(12000, [](Simulation& s) {
    s.radio().lossCentralToPeripheral = 0;
    s.radio().lossPeripheralToCentral = 0;
  }, "radio: loss off");
  sim.runUntil(15000);
  int reconnects = 0;
  for (const auto& e : sim.events())
    if (e.t >= 3000000 && e.t <= 12000000 && e.text.find("closed connection") != std::string::npos) ++reconnects;
  sim.log("check", "remote-initiated reconnects during loss: " + std::to_string(reconnects));
  FWSIM_CHECK(sim, reconnects <= 2, "occasional lost packets do not make the remote reconnect repeatedly");
  checkHealthy(sim, 13500, 15000, "after loss stops");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(millis_rollover, "Both boards' millis() wrap past 2^32 ms while linked") {
  boot(sim);
  // Start both clocks 4 s before millis() overflows (49.7 days of uptime).
  const uint64_t wrap = (uint64_t(1) << 32) * 1000ULL - 4000000ULL;
  sim.board(Robot).timeOffsetUs = wrap;
  sim.board(Remote).timeOffsetUs = wrap;
  sim.runUntil(9000);
  checkHealthy(sim, 3000, 9000, "across the millis() wrap at ~4 s");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(link_stability, "Perfect radio for 60 s: once healthy, the link must never drop") {
  boot(sim);
  sim.runUntil(60000);
  const double up = firstPermitted(sim);
  int drops = 0;
  double firstDrop = -1;
  for (const auto& e : sim.events())
    if (e.t > up * 1000 && e.text.find("closed connection") != std::string::npos) {
      if (firstDrop < 0) firstDrop = e.t / 1000.0;
      ++drops;
    }
  char buf[160];
  std::snprintf(buf, sizeof buf, "link healthy at %.0f ms; %d remote-initiated reconnect(s) afterwards%s",
                up, drops, drops ? " (see the first one in the replay)" : "");
  sim.log("check", buf);
  FWSIM_CHECK(sim, up > 0, "link becomes healthy");
  FWSIM_CHECK(sim, drops == 0, "no reconnects on a perfect radio after the first healthy link");
  checkWatchdogInvariant(sim);
}
