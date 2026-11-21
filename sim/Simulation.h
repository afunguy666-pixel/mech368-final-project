#ifndef MECH368_SIL_SIMULATION_H
#define MECH368_SIL_SIMULATION_H

#include <Mech368Protocol.h>
#include <array>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>

namespace sil {
namespace wire = mech368::ble;

constexpr uint32_t kStepMs = 10;
constexpr uint32_t kDelayMs = 20;
constexpr double kStepSeconds = 0.01;
constexpr double kPi = 3.14159265358979323846;
static_assert(wire::kSendMs == 50, "SIL scenarios assume the Phase 1 cadence");

enum class Scenario { Straight, Turn90, SensorError, Watchdog, Estop };

struct Pose {
  double x = 0.0, y = 0.0, yaw = 0.0; // yaw is unwrapped
  void step(double distance, double angle) {
    const double half = angle / 2.0;
    const double sinc = std::abs(half) < 1e-9 ? 1.0 : std::sin(half) / half;
    x += distance * sinc * std::cos(yaw + half);
    y += distance * sinc * std::sin(yaw + half);
    yaw += angle;
  }
};

struct Row {
  uint32_t t;
  bool connected, accepted, latched, allowed;
  int16_t linear, angular;
  Pose truth, encoder, fused;
  double leftRadPerSec, rightRadPerSec;
  const char* event;
};

class Noise {
 public:
  explicit Noise(uint32_t seed) : state_(seed ? seed : 1U) {}
  double symmetric() {
    state_ ^= state_ << 13;
    state_ ^= state_ >> 17;
    state_ ^= state_ << 5;
    return (static_cast<double>(state_ >> 8) / 8388608.0) - 1.0;
  }
 private:
  uint32_t state_;
};

struct ScheduledFrame {
  uint32_t at;
  std::array<uint8_t, wire::kFrameSize> bytes;
};

struct Config {
  // Virtual assumptions only; none are measured on the course robot.
  double trackMetres = 0.10;
  double wheelRadiusMetres = 0.020;
  unsigned encoderCountsPerRev = 1024;  // decoded counts at the wheel
  double leftScaleError = -0.02;
  double rightScaleError = 0.02;
  double encoderNoiseMetres = 0.000015;
  double gyroBiasRadPerSec = 0.002;
  double gyroNoiseRadPerSec = 0.006;
};

class Encoder {
 public:
  double observe(double distance, double metresPerCount) {
    fractionalCount_ += distance / metresPerCount;
    const double count = std::round(fractionalCount_);
    const double increment = (count - previousCount_) * metresPerCount;
    previousCount_ = count;
    return increment;
  }
 private:
  double fractionalCount_ = 0.0;
  double previousCount_ = 0.0;
};

struct Script {
  uint32_t end;
  bool sensorError;
};

inline Script settings(Scenario scenario) {
  switch (scenario) {
    case Scenario::Straight: return {2020, false};
    case Scenario::Turn90: return {2020, false};
    case Scenario::SensorError: return {7020, true};
    case Scenario::Watchdog: return {1700, false};
    case Scenario::Estop: return {1200, false};
  }
  throw std::invalid_argument("unknown SIL scenario");
}

inline std::vector<Row> run(Scenario scenario, uint32_t seed = 368U,
                            const Config& config = Config()) {
  if (!std::isfinite(config.trackMetres) || config.trackMetres <= 0.0 ||
      !std::isfinite(config.wheelRadiusMetres) || config.wheelRadiusMetres <= 0.0 ||
      config.encoderCountsPerRev == 0)
    throw std::invalid_argument("invalid SIL geometry or encoder resolution");
  const auto script = settings(scenario);
  std::vector<Row> rows;
  rows.reserve(script.end / kStepMs + 1U);
  wire::Receiver receiver;
  std::deque<ScheduledFrame> transit;
  Pose truth, encoder, fused;
  Noise noise(seed);
  Encoder leftEncoder, rightEncoder;
  const double metresPerCount = (2.0 * kPi * config.wheelRadiusMetres) /
                                config.encoderCountsPerRev;
  uint32_t session = 1;
  uint16_t sequence = 0;
  bool connected = true;
  receiver.connect(session, 0);

  for (uint32_t now = 0; now <= script.end; now += kStepMs) {
    const char* event = "";
    const bool watchdog = scenario == Scenario::Watchdog;
    const bool estop = scenario == Scenario::Estop;
    if ((watchdog && now == 1300) || (estop && now == 550)) {
      event = "disconnect";
      connected = false;
      receiver.disconnect(now);
      transit.clear();
    }
    if ((watchdog && now == 1450) || (estop && now == 700)) {
      event = "reconnect";
      connected = true;
      receiver.connect(++session, now);
      sequence = 0;
    }

    receiver.tick(now);
    // Only synthetic, correctly encoded local commands. Status/challenge is
    // sampled immediately; only command delivery has a simulated delay/loss.
    if (connected && now % wire::kSendMs == 0) {
      auto frame = receiver.status(now);
      frame.sequence = sequence++;
      frame.type = (estop && now == 300) ? wire::Type::Stop : wire::Type::Motion;
      frame.linear = frame.type == wire::Type::Stop ? 0 : 250;
      frame.angular = 0;
      if ((scenario == Scenario::Turn90 && now >= 1000) ||
          (script.sensorError && now >= 4000 && now < 5000)) {
        frame.linear = 0;
        frame.angular = 1571;
      }
      const bool drop = watchdog && now >= 550 && now < 1050;
      if (drop) event = "packet_dropped";
      if (!drop) {
        ScheduledFrame scheduled = {};
        scheduled.at = now + kDelayMs;
        if (!wire::encode(frame, scheduled.bytes.data(), scheduled.bytes.size()))
          throw std::logic_error("invalid scripted motion frame");
        transit.push_back(scheduled);
      }
    }
    bool accepted = false;
    while (!transit.empty() && transit.front().at <= now) {
      const auto& bytes = transit.front().bytes;
      const bool received = receiver.receive(bytes.data(), bytes.size(), now);
      accepted = received || accepted;
      event = !received ? "packet_rejected" :
              (estop && now == 300 + kDelayMs) ? "estop" : "command_accepted";
      transit.pop_front();
    }
    // Match the peripheral: receive and process E-stop before local recovery.
    // The recovered gate remains closed until a later valid motion packet.
    const bool recovery = estop && now == 900;
    receiver.tick(now, recovery);
    if (recovery) event = "local_recovery";
    const bool allowed = receiver.allowed(now);
    const bool latched = (receiver.status(now).linear & wire::EstopLatched) != 0;
    const int16_t linear = receiver.linear(now);
    const int16_t angular = receiver.angular(now);
    if (connected && !latched && !allowed && !rows.empty() && rows.back().allowed)
      event = "watchdog_timeout";

    // The trace at t records pose before integrating the command over [t,t+10).
    const double v = static_cast<double>(linear) / 1000.0;
    const double omega = static_cast<double>(angular) / 1000.0;
    const double leftVelocity = v - omega * config.trackMetres / 2.0;
    const double rightVelocity = v + omega * config.trackMetres / 2.0;
    rows.push_back({now, connected, accepted, latched, allowed, linear, angular,
                    truth, encoder, fused, leftVelocity / config.wheelRadiusMetres,
                    rightVelocity / config.wheelRadiusMetres, event});
    if (now == script.end) break;
    const double left = leftVelocity * kStepSeconds;
    const double right = rightVelocity * kStepSeconds;
    truth.step((left + right) / 2, (right - left) / config.trackMetres);
    const double noisyLeft = script.sensorError
      ? left * (1.0 + config.leftScaleError) + config.encoderNoiseMetres * noise.symmetric()
      : left;
    const double noisyRight = script.sensorError
      ? right * (1.0 + config.rightScaleError) + config.encoderNoiseMetres * noise.symmetric()
      : right;
    const double obsLeft = leftEncoder.observe(noisyLeft, metresPerCount);
    const double obsRight = rightEncoder.observe(noisyRight, metresPerCount);
    const double encoderYaw = (obsRight - obsLeft) / config.trackMetres;
    const double dist = (obsLeft + obsRight) / 2.0;
    const double gyroYaw = kStepSeconds * (omega + (script.sensorError
      ? config.gyroBiasRadPerSec + config.gyroNoiseRadPerSec * noise.symmetric()
      : 0.0));
    encoder.step(dist, encoderYaw);
    // Illustrative constant-weight fusion of instantaneous yaw increments.
    // The same wheel observations feed both estimators; the latter also sees gyro.
    fused.step(dist, 0.1 * encoderYaw + 0.9 * gyroYaw);
  }
  return rows;
}

} // namespace sil
#endif
