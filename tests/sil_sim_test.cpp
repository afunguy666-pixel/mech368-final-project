#include "../sim/Simulation.h"
#include <cassert>
#include <string>

namespace {
using Rows = std::vector<sil::Row>;
const sil::Row& at(const Rows& rows, uint32_t t) {
  const auto& row = rows.at(t / sil::kStepMs);
  assert(row.t == t);
  return row;
}

void blocked(const sil::Row& row) {
  assert(!row.allowed && row.linear == 0 && row.angular == 0);
  assert(row.leftRadPerSec == 0 && row.rightRadPerSec == 0);
}
void verifyGate(const Rows& rows) {
  for (const auto& row : rows)
    if (!row.allowed || !row.connected || row.latched) blocked(row);
}
double posError(const sil::Pose& estimate, const sil::Pose& truth) {
  return std::hypot(estimate.x - truth.x, estimate.y - truth.y);
}
} // namespace

int main() {
  const auto straight = sil::run(sil::Scenario::Straight);
  blocked(at(straight, 0));
  blocked(at(straight, 10));
  assert(at(straight, 20).accepted && at(straight, 20).linear == 250);
  assert(std::abs(straight.back().truth.x - 0.5) < 1e-9);
  assert(std::abs(straight.back().truth.y) < 1e-9);
  assert(posError(straight.back().encoder, straight.back().truth) < 0.0002);
  assert(posError(straight.back().fused, straight.back().truth) < 0.0002);

  const auto turn = sil::run(sil::Scenario::Turn90);
  const auto& pivot = turn.back();
  assert(std::abs(pivot.truth.yaw - sil::kPi / 2.0) < 0.001);
  const auto& beforePivot = at(turn, 1020);
  assert(std::abs(beforePivot.truth.x - 0.25) < 1e-9);
  assert(std::abs(beforePivot.truth.yaw) < 1e-9);
  assert(posError(pivot.truth, beforePivot.truth) < 1e-9);
  assert(std::abs(pivot.encoder.yaw - pivot.truth.yaw) < 0.002);
  assert(std::abs(pivot.fused.yaw - pivot.truth.yaw) < 0.002);

  const auto error = sil::run(sil::Scenario::SensorError);
  const auto repeat = sil::run(sil::Scenario::SensorError);
  const auto otherSeed = sil::run(sil::Scenario::SensorError, 369U);
  assert(error.back().encoder.x == repeat.back().encoder.x);
  assert(error.back().fused.yaw == repeat.back().fused.yaw);
  assert(error.back().fused.yaw != otherSeed.back().fused.yaw);
  const auto& noisy = error.back();
  assert(posError(noisy.encoder, noisy.truth) > 0.01);
  assert(std::isfinite(posError(noisy.fused, noisy.truth)));
  assert(std::isfinite(noisy.encoder.yaw) && std::isfinite(noisy.fused.yaw));

  const auto watchdog = sil::run(sil::Scenario::Watchdog);
  assert(at(watchdog, 520).accepted && at(watchdog, 520).allowed);
  const uint32_t expiry = 520U + mech368::MotionSafety::kTimeoutMs;
  for (uint32_t t = 530; t < expiry; t += sil::kStepMs) {
    assert(at(watchdog, t).connected && !at(watchdog, t).accepted && at(watchdog, t).allowed);
  }
  assert(std::string(at(watchdog, expiry).event) == "watchdog_timeout");
  for (uint32_t t = expiry; t < 1070U; t += sil::kStepMs) {
    assert(at(watchdog, t).connected && !at(watchdog, t).accepted);
    blocked(at(watchdog, t));
  }
  assert(at(watchdog, 1070).accepted && at(watchdog, 1070).allowed);
  assert(at(watchdog, 1290).allowed);
  assert(!at(watchdog, 1300).connected);
  blocked(at(watchdog, 1300));
  assert(at(watchdog, 1450).connected);
  blocked(at(watchdog, 1450));
  blocked(at(watchdog, 1460));
  assert(at(watchdog, 1470).accepted && at(watchdog, 1470).allowed);

  const auto stop = sil::run(sil::Scenario::Estop);
  assert(at(stop, 310).allowed && at(stop, 310).linear == 250);
  assert(at(stop, 320).accepted);
  assert(!at(stop, 550).connected && at(stop, 700).connected);
  assert(at(stop, 720).accepted); // received motion must not clear E-stop
  for (uint32_t t = 320; t < 900; t += sil::kStepMs) {
    assert(at(stop, t).latched);
    blocked(at(stop, t));
  }
  assert(!at(stop, 900).latched);
  blocked(at(stop, 900));
  blocked(at(stop, 910));
  assert(at(stop, 920).accepted && at(stop, 920).allowed && at(stop, 920).linear == 250);

  verifyGate(straight); verifyGate(turn); verifyGate(error);
  verifyGate(watchdog); verifyGate(stop);
}
