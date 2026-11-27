// Low-battery decision from the A7 divider. Portable C++11, host-tested in
// tests/control_test.cpp.
//
// pack mV = ADC * adcRefMv / adcMax * divider, low-pass filtered (tau).
// The filtered value must stay below lowMv for lowHoldMs before LOW is
// declared, and above clearMv for clearHoldMs before it is released, so a
// motor start sag or a single noisy read cannot flash the indicator. A
// reading below absentMv (USB-powered bench, battery switch off) is reported
// as ABSENT, not LOW. The monitor only reports; it never cuts power.
//
// All defaults are PROVISIONAL until measured with a multimeter on the real
// pack and divider (see docs/GESTURE_BATTERY_HEATMAP.zh-CN.md).
#ifndef MECH368_BATTERY_MONITOR_H
#define MECH368_BATTERY_MONITOR_H

#include <stdint.h>

namespace mech368 {
namespace control {

struct BatteryConfig {
  float adcRefMv = 3300.0f;   // Nano 33 BLE analogRead reference (3.3 V rail)
  float adcMax = 1023.0f;     // 10-bit default resolution
  float divider = 3.0f;       // Vpack / VA7, PROVISIONAL
  float lowMv = 6400.0f;      // PROVISIONAL, 7.2 V-class pack assumed
  float clearMv = 6800.0f;    // PROVISIONAL, must be > lowMv
  float absentMv = 1500.0f;   // below this no pack is connected
  float tauMs = 1000.0f;      // low-pass time constant
  uint32_t settleMs = 2000;   // ignore the first samples after boot
  uint32_t lowHoldMs = 5000;
  uint32_t clearHoldMs = 10000;
  uint32_t absentHoldMs = 1000;
};

enum class BatteryState : uint8_t { Unknown = 0, Ok = 1, Low = 2, Absent = 3 };

inline const char* batteryStateName(BatteryState s) {
  switch (s) {
    case BatteryState::Unknown: return "unknown";
    case BatteryState::Ok: return "ok";
    case BatteryState::Low: return "LOW";
    case BatteryState::Absent: return "absent";
  }
  return "?";
}

class BatteryMonitor {
 public:
  explicit BatteryMonitor(const BatteryConfig& cfg = BatteryConfig()) : cfg_(cfg) {}
  const BatteryConfig& config() const { return cfg_; }

  float toMillivolts(uint16_t adc) const { return adc * (cfg_.adcRefMv / cfg_.adcMax) * cfg_.divider; }

  void sample(uint32_t ms, uint16_t adc) {
    const float mv = toMillivolts(adc);
    if (!started_) {
      started_ = true;
      startMs_ = lastMs_ = ms;
      filtered_ = mv;
    } else {
      const float dt = static_cast<float>(static_cast<uint32_t>(ms - lastMs_));
      lastMs_ = ms;
      filtered_ += (dt / (cfg_.tauMs + dt)) * (mv - filtered_);
    }
    const Zone zone = classify(filtered_);
    if (zone != zone_ || !zoneValid_) {
      zone_ = zone;
      zoneSince_ = ms;
      zoneValid_ = true;
    }
    if (static_cast<uint32_t>(ms - startMs_) < cfg_.settleMs) return;
    const uint32_t inZone = ms - zoneSince_;
    if (zone_ == Zone::Absent) {
      if (inZone >= cfg_.absentHoldMs) state_ = BatteryState::Absent;
      return;
    }
    switch (state_) {
      case BatteryState::Unknown:
      case BatteryState::Absent:
        if (zone_ != Zone::Low) state_ = BatteryState::Ok;
        else if (inZone >= cfg_.lowHoldMs) state_ = BatteryState::Low;
        break;
      case BatteryState::Ok:
        if (zone_ == Zone::Low && inZone >= cfg_.lowHoldMs) state_ = BatteryState::Low;
        break;
      case BatteryState::Low:
        if (zone_ == Zone::High && inZone >= cfg_.clearHoldMs) state_ = BatteryState::Ok;
        break;
    }
  }

  BatteryState state() const { return state_; }
  bool low() const { return state_ == BatteryState::Low; }
  float filteredMv() const { return filtered_; }

 private:
  enum class Zone : uint8_t { Absent, Low, Mid, High };
  Zone classify(float mv) const {
    if (mv < cfg_.absentMv) return Zone::Absent;
    if (mv < cfg_.lowMv) return Zone::Low;
    if (mv > cfg_.clearMv) return Zone::High;
    return Zone::Mid;
  }
  BatteryConfig cfg_;
  BatteryState state_ = BatteryState::Unknown;
  Zone zone_ = Zone::Mid;
  bool started_ = false, zoneValid_ = false;
  uint32_t startMs_ = 0, lastMs_ = 0, zoneSince_ = 0;
  float filtered_ = 0.0f;
};

}  // namespace control
}  // namespace mech368
#endif
