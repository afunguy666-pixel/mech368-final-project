// Host tests for libraries/Mech368Line: sensor scaling and line position,
// PD follower signs, kinematics, odometry, gyro bias and the navigator's
// corner / lost-line / end / obstacle-detour state machine.
#include <Mech368Line.h>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace mech368::line;

namespace {
bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

// Four sensors at -4.5, -1.5, +1.5, +4.5 cm (index 0 = rightmost).
const float kY[4] = {-0.045f, -0.015f, 0.015f, 0.045f};

LineReading digital(LineSensorArray& a, bool r0, bool r1, bool r2, bool r3) {
  const bool b[4] = {r0, r1, r2, r3};
  return a.readDigital(b);
}

void testSensors() {
  LineSensorArray a;
  assert(!a.configure(1, kY));
  assert(a.configure(4, kY));
  assert(a.leftOuter() == 3 && a.rightOuter() == 0);

  // Calibration: floor ~120, tape ~880 counts; black reads high.
  assert(!a.calibrationValid(100));
  const uint16_t floorRaw[4] = {120, 130, 110, 125}, tapeRaw[4] = {880, 900, 870, 860};
  a.calibrate(floorRaw);
  a.calibrate(tapeRaw);
  assert(a.calibrationValid(300));
  assert(near(a.normalize(0, 120), 0.0f, 1e-6f) && near(a.normalize(0, 880), 1.0f, 1e-6f));
  assert(near(a.normalize(0, 500), 0.5f, 1e-3f));
  assert(near(a.normalize(0, 2000), 1.0f, 1e-6f) && near(a.normalize(0, 0), 0.0f, 1e-6f));

  // Centred line: the two inner sensors on the tape.
  const uint16_t centred[4] = {120, 900, 870, 125};
  LineReading r = a.readRaw(centred);
  assert(r.onLine && r.blackCount == 2 && near(r.position, 0.0f, 1e-3f));
  // Line to the left: weighted toward +y.
  const uint16_t left[4] = {120, 300, 870, 860};
  r = a.readRaw(left);
  assert(r.onLine && r.position > 0.015f && r.position < 0.045f);
  // Lost after seeing it on the left: extrapolate past the left sensor.
  const uint16_t floorOnly[4] = {120, 130, 110, 125};
  r = a.readRaw(floorOnly);
  assert(!r.onLine && r.position > 0.045f && r.lastSeen > 0.015f);

  // Inverted polarity (black reads low).
  LineSensorArray inv;
  inv.configure(4, kY);
  inv.blackIsHigh = false;
  for (int i = 0; i < 4; ++i) inv.setCalibration(i, 900, 100);
  const uint16_t rightSide[4] = {100, 100, 900, 900};
  r = inv.readRaw(rightSide);
  assert(r.onLine && r.position < -0.015f);

  // Digital (LM393) readings: quantised positions.
  LineSensorArray d;
  d.configure(4, kY);
  assert(near(digital(d, false, true, true, false).position, 0.0f, 1e-6f));
  assert(near(digital(d, false, true, false, false).position, -0.015f, 1e-6f));
  assert(near(digital(d, true, true, false, false).position, -0.030f, 1e-6f));
  r = digital(d, false, false, false, false);
  assert(!r.onLine && r.position < -0.045f);  // last seen on the right
  r = digital(d, true, true, true, false);
  assert(r.blackCount == 3 && r.rightOuterBlack && !r.leftOuterBlack);
}

void testFollower() {
  LineFollower f;
  Twist t = f.update(0.0f, 0.005f);
  assert(near(t.omega, 0.0f, 1e-6f) && near(t.v, f.cruise, 1e-6f));
  f.reset();
  t = f.update(0.02f, 0.005f);  // line left -> turn left, slow down
  assert(t.omega > 0.0f && t.v < f.cruise && t.v >= f.minSpeed);
  f.reset();
  t = f.update(-0.02f, 0.005f);
  assert(t.omega < 0.0f);
  f.reset();
  t = f.update(1.0f, 0.005f);  // saturates
  assert(near(t.omega, f.maxOmega, 1e-6f) && near(t.v, f.minSpeed, 1e-6f));
  // Derivative opposes a fast approach to the centre (damping).
  f.reset();
  f.update(0.02f, 0.005f);
  const float withD = f.update(0.015f, 0.005f).omega;
  assert(withD < f.kp * 0.015f);
}

void testKinematics() {
  const float W = 0.11f;
  WheelSpeeds w = toWheels(Twist(0.2f, 0.0f), W);
  assert(near(w.left, 0.2f, 1e-6f) && near(w.right, 0.2f, 1e-6f));
  w = toWheels(Twist(0.0f, 2.0f), W);  // pivot left: right wheel forward
  assert(near(w.left, -0.11f, 1e-6f) && near(w.right, 0.11f, 1e-6f));
  const Twist back = toTwist(toWheels(Twist(0.15f, -1.5f), W), W);
  assert(near(back.v, 0.15f, 1e-6f) && near(back.omega, -1.5f, 1e-6f));
  // Limit keeps the turn rate and drops forward speed.
  Twist lim = limitTwist(Twist(0.4f, 4.0f), W, 0.45f);
  w = toWheels(lim, W);
  assert(near(lim.omega, 4.0f, 1e-6f) && std::fabs(w.right) <= 0.45f + 1e-5f);
  lim = limitTwist(Twist(0.4f, 20.0f), W, 0.45f);
  assert(near(lim.v, 0.0f, 1e-6f) && near(lim.omega, 2.0f * 0.45f / W, 1e-4f));

  assert(near(wrapAngle(3.5f * kPi), -0.5f * kPi, 1e-5f));
  assert(near(wrapAngle(-kPi), kPi, 1e-5f));
}

void testOdometry() {
  Odometry o;
  o.reset(Pose(0, 0, 0));
  for (int i = 0; i < 100; ++i) o.update(0.01f, 0.01f, 0.1f);
  assert(near(o.pose().x, 1.0f, 1e-4f) && near(o.pose().y, 0.0f, 1e-5f) && near(o.distance(), 1.0f, 1e-4f));
  // Quarter circle of radius 0.5 m to the left using gyro heading.
  o.reset(Pose(0, 0, 0));
  const int n = 400;
  const float arc = 0.5f * kPi / 2.0f;
  for (int i = 0; i < n; ++i) o.updateWithGyro(arc / n, arc / n, (kPi / 2.0f) / n);
  assert(near(o.pose().x, 0.5f, 2e-3f) && near(o.pose().y, 0.5f, 2e-3f) &&
         near(o.pose().heading, kPi / 2.0f, 1e-4f));

  GyroBias g;
  for (int i = 0; i < 200; ++i) g.add(0.2f + ((i % 2) ? 0.05f : -0.05f));
  g.add(40.0f);  // bump while calibrating: rejected
  assert(g.ready() && near(g.bias(), 0.2f, 1e-3f));

  float roll, pitch;
  rollPitchFromAccel(-std::sin(0.3f), std::sin(0.2f) * std::cos(0.3f), std::cos(0.2f) * std::cos(0.3f), roll, pitch);
  assert(near(roll, 0.2f, 1e-5f) && near(pitch, 0.3f, 1e-5f));

  WheelSpeedPI pi;
  assert(pi.update(0.0f, 0.1f, 0.01f) == 0.0f);
  const float duty = pi.update(0.2f, 0.1f, 0.01f);
  assert(duty > pi.feedForward * 0.2f && duty <= 1.0f);
}

// ---- navigator: a tiny scripted world ----
struct Script {
  LineNavigator nav;
  LineSensorArray arr;
  NavInputs in;
  Script() {
    arr.configure(4, kY);
    in.dt = 0.005f;
  }
  void line(bool r0, bool r1, bool r2, bool r3) { in.line = digital(arr, r0, r1, r2, r3); }
  // Advance odometry/heading by the commanded twist and run one step.
  Twist step() {
    const Twist t = nav.update(in);
    in.distance += t.v * in.dt;
    in.heading = wrapAngle(in.heading + t.omega * in.dt);
    return t;
  }
  int runUntil(NavState s, int maxSteps) {
    for (int i = 0; i < maxSteps; ++i) {
      if (nav.state() == s) return i;
      step();
    }
    return -1;
  }
};

void testCorner() {
  Script s;
  s.line(false, true, true, false);
  s.nav.start(s.in);
  assert(s.nav.state() == NavState::Follow);
  for (int i = 0; i < 100; ++i) assert(s.step().v > 0.0f);  // arm the corner detector
  // Corner to the right: inner pair plus the right outer sensor.
  s.line(true, true, true, false);
  assert(s.runUntil(NavState::CornerAdvance, 5) >= 0);
  // Advance over white, then pivot right until the line is centred again.
  s.line(false, false, false, false);
  assert(s.runUntil(NavState::CornerPivot, 200) >= 0);
  const float start = s.in.heading;
  Twist t = s.step();
  assert(t.v == 0.0f && t.omega < 0.0f);  // pivot right
  while (wrapAngle(start - s.in.heading) < degToRad(80)) s.step();
  s.line(false, true, true, false);
  assert(s.runUntil(NavState::Settle, 5) >= 0);
  assert(s.runUntil(NavState::Follow, 100) >= 0);
  assert(s.nav.corners() == 1);
}

void testCornerNeedsMinPivot() {
  Script s;
  s.line(false, true, true, false);
  s.nav.start(s.in);
  for (int i = 0; i < 100; ++i) s.step();
  s.line(false, true, true, true);  // corner to the left
  assert(s.runUntil(NavState::CornerPivot, 200) >= 0);
  // Line still centred from the old leg: must not finish before minPivot.
  s.line(false, true, true, false);
  for (int i = 0; i < 20; ++i) s.step();
  assert(s.nav.state() == NavState::CornerPivot);
  assert(s.runUntil(NavState::Settle, 400) >= 0);
  assert(wrapAngle(s.in.heading) >= s.nav.cfg.minPivot - 1e-3f);
}

// Narrow array: wide black that is only an offset must not start a corner.
void testCornerConfirmByLoss() {
  const float y[4] = {-0.030f, -0.010f, 0.010f, 0.030f};
  Script s;
  s.arr.configure(4, y);
  s.nav.cfg.confirmCornerByLoss = true;
  s.nav.cfg.cornerAdvanceAfterLoss = 0.053f;
  s.line(false, true, true, false);
  s.nav.start(s.in);
  for (int i = 0; i < 100; ++i) s.step();
  // Robot 8 mm off centre: three sensors black, one outer. Line continues.
  s.line(true, true, true, false);
  assert(s.runUntil(NavState::CornerCheck, 5) >= 0);
  assert(s.runUntil(NavState::Follow, 400) >= 0);  // drove 7 cm, line never ended
  assert(s.nav.corners() == 0);
  s.line(false, true, true, false);                // PD re-centred
  for (int i = 0; i < 200; ++i) s.step();          // re-arm
  // A cone during the corner check still starts a detour.
  s.line(true, true, true, false);
  assert(s.runUntil(NavState::CornerCheck, 5) >= 0);
  s.in.obstacleLeft = true;
  assert(s.runUntil(NavState::AvoidBrake, 5) >= 0);
  s.in.obstacleLeft = false;
  s.nav.stop();
  s.line(false, true, true, false);
  s.nav.start(s.in);
  for (int i = 0; i < 100; ++i) s.step();
  // Real corner: wide, then the line ends under the bar.
  s.line(true, true, true, false);
  assert(s.runUntil(NavState::CornerCheck, 5) >= 0);
  for (int i = 0; i < 20; ++i) s.step();
  s.line(false, false, false, false);
  assert(s.runUntil(NavState::CornerAdvance, 5) >= 0);
  const float from = s.in.distance;
  assert(s.runUntil(NavState::CornerPivot, 400) >= 0);
  assert(near(s.in.distance - from, 0.053f, 0.003f));
  while (wrapAngle(-s.in.heading) < degToRad(80)) s.step();
  s.line(false, true, true, false);
  assert(s.runUntil(NavState::Settle, 5) >= 0);
  assert(s.nav.corners() == 1);
}

void testLostAndEnd() {
  // Lost to the left -> search left, then find it.
  Script s;
  s.line(false, false, true, true);
  s.nav.start(s.in);
  s.step();
  s.line(false, false, false, false);
  assert(s.runUntil(NavState::Search, 10) >= 0);
  assert(s.step().omega > 0.0f);
  s.line(false, true, true, false);
  assert(s.runUntil(NavState::Settle, 10) >= 0);

  // Lost straight ahead -> creep, then finish.
  Script e;
  e.line(false, true, true, false);
  e.nav.start(e.in);
  e.step();
  e.line(false, false, false, false);
  assert(e.runUntil(NavState::EndCheck, 10) >= 0);
  assert(e.runUntil(NavState::Finished, 400) >= 0);
  assert(e.step().v == 0.0f);

  // Nothing anywhere -> full sweep, then Lost (stopped).
  Script l;
  l.line(false, false, false, true);
  l.nav.start(l.in);
  l.step();
  l.line(false, false, false, false);
  assert(l.runUntil(NavState::Lost, 5000) >= 0);
  assert(!l.nav.running());
}

void testDetour() {
  Script s;
  s.line(false, true, true, false);
  s.nav.start(s.in);
  for (int i = 0; i < 100; ++i) s.step();
  // Cone seen by the left sensor -> detour to the right (clockwise).
  s.in.obstacleLeft = true;
  assert(s.runUntil(NavState::AvoidBrake, 5) >= 0);
  assert(s.nav.detours() == 1);
  s.in.obstacleLeft = false;
  s.line(false, false, false, false);
  assert(s.runUntil(NavState::AvoidLegOut, 400) >= 0);
  assert(near(wrapAngle(s.in.heading), -s.nav.cfg.turnOut, s.nav.cfg.headingTolerance + 0.02f));
  assert(s.runUntil(NavState::AvoidPass, 2000) >= 0);
  assert(near(wrapAngle(s.in.heading), 0.0f, s.nav.cfg.headingTolerance + 0.02f));
  assert(s.runUntil(NavState::AvoidSeek, 4000) >= 0);
  assert(near(wrapAngle(s.in.heading), s.nav.cfg.turnIn, s.nav.cfg.headingTolerance + 0.02f));
  // Line found: advance, then pivot right (the detour side) onto it.
  s.line(false, true, true, true);
  assert(s.runUntil(NavState::RejoinPivot, 200) >= 0);
  s.line(false, false, false, false);
  assert(s.step().omega < 0.0f);
  while (wrapAngle(s.in.heading) > 0.1f) s.step();
  s.line(false, true, true, false);
  assert(s.runUntil(NavState::Settle, 5) >= 0);
  assert(s.runUntil(NavState::Follow, 100) >= 0);

  // Blocked on the way out: back up, retry on the other side; blocked again:
  // stop for a human.
  Script b;
  b.line(false, true, true, false);
  b.nav.start(b.in);
  for (int i = 0; i < 100; ++i) b.step();
  b.in.obstacleRight = true;
  assert(b.runUntil(NavState::AvoidBrake, 5) >= 0);
  b.in.obstacleRight = false;
  b.line(false, false, false, false);
  assert(b.runUntil(NavState::AvoidLegOut, 400) >= 0);
  assert(wrapAngle(b.in.heading) > 0.5f);  // detouring left
  for (int i = 0; i < 40; ++i) b.step();
  const float outAt = b.in.distance;
  b.in.obstacleLeft = true;
  assert(b.runUntil(NavState::AvoidRetreat, 10) >= 0);
  b.in.obstacleLeft = false;
  assert(b.step().v < 0.0f);
  assert(b.runUntil(NavState::AvoidTurnOut, 400) >= 0);
  assert(b.in.distance < outAt - 0.02f);  // backed up
  assert(b.runUntil(NavState::AvoidLegOut, 400) >= 0);
  assert(wrapAngle(b.in.heading) < -0.5f);  // now detouring right
  b.in.obstacleRight = true;
  assert(b.runUntil(NavState::Blocked, 10) >= 0);
  assert(b.step().v == 0.0f);
}
}  // namespace

int main() {
  testSensors();
  testFollower();
  testKinematics();
  testOdometry();
  testCorner();
  testCornerNeedsMinPivot();
  testCornerConfirmByLoss();
  testLostAndEnd();
  testDetour();
  std::puts("line_nav_test: all assertions passed");
  return 0;
}
