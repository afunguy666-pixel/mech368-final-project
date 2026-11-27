// Host tests for libraries/Mech368Control: tilt steering, drive mode
// selection and the BLE command fields they use.
#include <DriveModeSwitch.h>
#include <Mech368Protocol.h>
#include <TiltSteering.h>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace mech368;
using namespace mech368::control;

namespace {
const float kDeg = 3.14159265f / 180.0f;

// Accelerometer reading (g) for a board at roll/pitch degrees, the inverse of
// line::rollPitchFromAccel (same model as the simulator's scripted hand).
void accel(float rollDeg, float pitchDeg, float& ax, float& ay, float& az) {
  const float r = rollDeg * kDeg, p = pitchDeg * kDeg;
  ax = -std::sin(p);
  ay = std::sin(r) * std::cos(p);
  az = std::cos(r) * std::cos(p);
}

// Feed samples at 100 Hz from t to t + ms, calling update() each sample.
line::Twist hold(TiltSteering& t, uint32_t& now, uint32_t ms, float roll, float pitch) {
  line::Twist out;
  for (uint32_t i = 0; i < ms; i += 10, now += 10) {
    float ax, ay, az;
    accel(roll, pitch, ax, ay, az);
    t.sample(now, ax, ay, az);
    out = t.update(now);
  }
  return out;
}

// Advance time without samples (frozen IMU).
line::Twist silent(TiltSteering& t, uint32_t& now, uint32_t ms) {
  line::Twist out;
  for (uint32_t i = 0; i < ms; i += 10, now += 10) out = t.update(now);
  return out;
}

bool near(float a, float b, float tol = 1e-3f) { return std::fabs(a - b) <= tol; }

void tiltZeroAndDirection() {
  TiltSteering t;
  uint32_t now = 1000;
  // No IMU: always zero, state reported.
  assert(t.state(now) == TiltState::NoImu);
  t.setImuReady(true);
  // A board held at a modest angle when switched on: that grip becomes zero.
  line::Twist out = hold(t, now, 300, 8, -12);
  assert(t.state(now) == TiltState::Zeroing && out.v == 0 && out.omega == 0);
  out = hold(t, now, 400, 8, -12);
  assert(t.zeroed() && near(t.zeroRollDeg(), 8, 0.01f) && near(t.zeroPitchDeg(), -12, 0.01f));
  assert(out.v == 0 && out.omega == 0);
  // Inside the deadband about the zero: still zero.
  out = hold(t, now, 500, 8 + 4, -12 - 4);
  assert(out.v == 0 && out.omega == 0 && t.state(now) == TiltState::Active);
  // Front edge down (pitch +) drives forward; slew limits the ramp.
  out = hold(t, now, 50, 8, -12 + 25);
  assert(out.v > 0 && out.v < 0.05f);
  out = hold(t, now, 1500, 8, -12 + 25);
  assert(near(out.v, t.config().maxLinear) && out.omega == 0);
  // Beyond full scale: limited.
  out = hold(t, now, 500, 8, -12 + 60);
  assert(near(out.v, t.config().maxLinear));
  // Back edge down reverses.
  out = hold(t, now, 2000, 8, -12 - 15);
  assert(out.v < 0 && near(out.v, -t.config().maxLinear * (15 - 5) / 20.0f, 0.01f));
  // Right edge down (roll +) turns right: omega < 0. Left edge down: omega > 0.
  out = hold(t, now, 2000, 8 + 25, -12);
  assert(near(out.v, 0) && near(out.omega, -t.config().maxAngular));
  out = hold(t, now, 2000, 8 - 25, -12);
  assert(near(out.omega, t.config().maxAngular));

  // Direction is configurable.
  TiltConfig flipped;
  flipped.pitchSign = -1;
  flipped.rollSign = 1;
  TiltSteering f(flipped);
  f.setImuReady(true);
  uint32_t t2 = 0;
  hold(f, t2, 1000, 0, 0);
  out = hold(f, t2, 2000, 25, 25);
  assert(near(out.v, -flipped.maxLinear) && near(out.omega, flipped.maxAngular));
}

void tiltZeroingRules() {
  TiltSteering t;
  t.setImuReady(true);
  uint32_t now = 0;
  // Moving hand: the zero never completes, output stays 0.
  for (int i = 0; i < 200; ++i) {
    float ax, ay, az;
    accel((i % 2) ? 5.0f : -5.0f, 0, ax, ay, az);
    t.sample(now, ax, ay, az);
    assert(t.update(now).v == 0);
    now += 10;
  }
  assert(!t.zeroed());
  // Too far from flat (e.g. board on its side): refused.
  hold(t, now, 1000, 50, 0);
  assert(!t.zeroed());
  // Re-zero on request: output drops to 0 at once and re-arms at the new zero.
  hold(t, now, 1000, 0, 0);
  assert(t.zeroed());
  line::Twist out = hold(t, now, 2000, 0, 25);
  assert(out.v > 0.2f);
  t.requestZero();
  out = t.update(now);
  assert(out.v == 0 && out.omega == 0 && !t.zeroed());
  // The operator still tips the board while re-zeroing is pending: the new
  // zero is the current grip, so the result is 0, not a reversal.
  out = hold(t, now, 1000, 0, 25);
  assert(t.zeroed() && out.v == 0);
}

void tiltImuFailure() {
  TiltSteering t;
  t.setImuReady(true);
  uint32_t now = 0;
  hold(t, now, 1000, 0, 0);
  line::Twist out = hold(t, now, 2000, 0, 25);
  assert(out.v > 0.2f);
  // Frozen IMU: zero within staleMs, without slewing down.
  out = silent(t, now, 90);
  assert(out.v > 0.2f);
  out = silent(t, now, 20);
  assert(out.v == 0 && out.omega == 0 && t.state(now) == TiltState::Stale);
  // Data returns while the board is still tipped: no lurch until neutral.
  out = hold(t, now, 1000, 0, 25);
  assert(out.v == 0 && t.state(now) == TiltState::WaitNeutral);
  hold(t, now, 200, 0, 0);
  assert(t.state(now) == TiltState::Active);
  out = hold(t, now, 2000, 0, 25);
  assert(out.v > 0.2f);
  // Implausible samples (shaking, NaN) are skipped, so they cannot refresh.
  for (int i = 0; i < 20; ++i, now += 10) {
    t.sample(now, 0.0f, 0.0f, 2.5f);
    t.sample(now, NAN, 0.0f, 1.0f);
    out = t.update(now);
  }
  assert(out.v == 0);
  // IMU reported lost.
  hold(t, now, 200, 0, 0);
  hold(t, now, 2000, 0, 25);
  t.setImuReady(false);
  out = t.update(now);
  assert(out.v == 0 && t.state(now) == TiltState::NoImu);
  // millis() wrap does not stall or jump.
  TiltSteering w;
  w.setImuReady(true);
  uint32_t wrap = UINT32_MAX - 1500;
  hold(w, wrap, 1000, 0, 0);
  out = hold(w, wrap, 2000, 0, 25);
  assert(near(out.v, w.config().maxLinear));
}

void protocolCarriesClampedTwist() {
  ble::RemoteLink link;
  ble::Frame status;
  status.type = ble::Type::Status;
  status.session = 7;
  status.challenge = 3;
  uint8_t bytes[ble::kFrameSize];
  assert(ble::encode(status, bytes, sizeof(bytes)));
  assert(link.status(bytes, sizeof(bytes), 0));
  ble::Frame f = link.command(false, 250, -2000);
  assert(f.type == ble::Type::Motion && f.linear == 250 && f.angular == -2000 && ble::valid(f));
  f = link.command(false, 30000, -30000);
  assert(f.linear == ble::kMaxLinear && f.angular == -ble::kMaxAngular && ble::valid(f));
  f = link.command(true, 250, 100);
  assert(f.type == ble::Type::Stop && f.linear == 0 && f.angular == 0 && ble::valid(f));

  // Robot side: an accepted command is readable only while MotionSafety
  // permits motion; a duplicate cannot extend it.
  ble::Receiver rx;
  rx.connect(7, 0);
  ble::Frame m = rx.status(0);
  ble::Frame cmd;
  cmd.type = ble::Type::Motion;
  cmd.session = 7;
  cmd.challenge = m.challenge;
  cmd.sequence = 1;
  cmd.linear = 200;
  cmd.angular = -500;
  assert(ble::encode(cmd, bytes, sizeof(bytes)));
  assert(rx.receive(bytes, sizeof(bytes), 10));
  assert(rx.linear(10) == 200 && rx.angular(10) == -500);
  assert(!rx.receive(bytes, sizeof(bytes), 300));  // duplicate: rejected
  rx.tick(10 + MotionSafety::kTimeoutMs);
  assert(rx.linear(10 + MotionSafety::kTimeoutMs) == 0 && rx.angular(10 + MotionSafety::kTimeoutMs) == 0);
}

ModeInputs inputs(Gesture g, bool driving = false, bool permitted = true, bool neutral = true,
                  bool latched = false) {
  ModeInputs in;
  in.gesture = g;
  in.driving = driving;
  in.motionPermitted = permitted;
  in.manualNeutral = neutral;
  in.latched = latched;
  return in;
}

void modeSwitching() {
  DriveModeSwitch m;
  assert(m.selected() == DriveMode::Idle);
  // UP without a selection never starts.
  assert(m.update(inputs(Gesture::Up)) == ModeAction::None);
  // Selecting never starts.
  assert(m.update(inputs(Gesture::Left)) == ModeAction::None && m.selected() == DriveMode::Manual);
  // MANUAL needs fresh heartbeats and a level handheld.
  assert(m.update(inputs(Gesture::Up, false, false)) == ModeAction::None);
  assert(m.update(inputs(Gesture::Up, false, true, false)) == ModeAction::None);
  assert(m.update(inputs(Gesture::Up)) == ModeAction::StartManual);
  // UP while driving does nothing; switching to AUTO stops first, no start.
  assert(m.update(inputs(Gesture::Up, true)) == ModeAction::None);
  assert(m.update(inputs(Gesture::Right, true)) == ModeAction::Stop && m.selected() == DriveMode::Auto);
  assert(m.update(inputs(Gesture::None)) == ModeAction::None);
  // AUTO does not care about the handheld tilt.
  assert(m.update(inputs(Gesture::Up, false, true, false)) == ModeAction::StartAuto);
  // Re-selecting the running mode also stops.
  assert(m.update(inputs(Gesture::Right, true)) == ModeAction::Stop);
  // DOWN always stops.
  assert(m.update(inputs(Gesture::Down, true)) == ModeAction::Stop);
  assert(m.update(inputs(Gesture::Down, false)) == ModeAction::Stop);
}

void estopAndReconnectNeverStart() {
  DriveModeSwitch m;
  m.update(inputs(Gesture::Left));
  // While latched, every gesture is consumed by the unlock: no selection
  // change, no stop/start action from this switch.
  const Gesture all[] = {Gesture::Up, Gesture::Down, Gesture::Left, Gesture::Right};
  for (Gesture g : all) {
    assert(m.update(inputs(g, false, false, true, true)) == ModeAction::None);
    assert(m.selected() == DriveMode::Manual);
  }
  // After the latch is cleared and heartbeats are fresh again (reconnect),
  // nothing happens without a separate UP.
  assert(m.update(inputs(Gesture::None)) == ModeAction::None);
  assert(m.update(inputs(Gesture::Up, false, true, false)) == ModeAction::None);  // tilted
  assert(m.update(inputs(Gesture::Up)) == ModeAction::StartManual);
}
}  // namespace

int main() {
  tiltZeroAndDirection();
  tiltZeroingRules();
  tiltImuFailure();
  protocolCarriesClampedTwist();
  modeSwitching();
  estopAndReconnectNeverStart();
  std::puts("control tests passed");
}
