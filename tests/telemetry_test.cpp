#include <DriveCalibration.h>
#include <Mech368Telemetry.h>
#include <cassert>
#include <cmath>

namespace tel = mech368::telemetry;
int main() {
  using mech368::calibration::Wheel;
  Wheel l{-1, 500, 0.200f}, r{1, 600, 0.204f};
  mech368::line::Odometry odom;
  odom.updateWithGyro(l.travel(0, 510), r.travel(600, 0), 0);
  assert(std::fabs(odom.pose().x - 0.204f) < 1e-5f && odom.pose().y == 0);
  odom.updateWithGyro(-0.1f, 0.1f, mech368::line::kPi / 2);
  assert(std::fabs(odom.pose().heading - mech368::line::kPi / 2) < 1e-6f);
  assert(std::fabs(odom.pose().x - 0.204f) < 1e-5f && odom.pose().y == 0);
  assert(std::fabs(r.travel(3, UINT32_MAX - 2) - r.travel(6, 0)) < 1e-8f);
  tel::Schedule schedule;
  const uint32_t start = UINT32_MAX - 70;
  schedule.reset(start);
  tel::Kind kind;
  uint16_t seq = 99;
  assert(schedule.due(start + 50, kind, seq) && kind == tel::Pose && seq == 0);
  assert(!schedule.due(start + 51, kind, seq));
  assert(schedule.due(start + 3050, kind, seq) && kind == tel::Pose && seq == 30);
  uint8_t p[20] = {};
  tel::header(p, tel::Pose, tel::Driving, 65535, 12345);
  tel::finish(p);
  assert(tel::valid(p, sizeof(p)));
  p[7] ^= 1;
  assert(!tel::valid(p, sizeof(p)) && !tel::valid(p, 19));
  uint8_t flags = 0;
  assert(tel::scaled16(33000, flags) == tel::kMissing && (flags & tel::Clipped));
}
