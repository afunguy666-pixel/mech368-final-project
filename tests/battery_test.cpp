// Host tests for libraries/Mech368Control/src/BatteryMonitor.h.
#include <BatteryMonitor.h>
#include <cassert>
#include <cmath>
#include <cstdio>

using namespace mech368::control;

namespace {
// ADC count for a pack voltage with the default 3.3 V reference and 3:1 divider.
uint16_t adcFor(const BatteryMonitor& m, float packMv) {
  const BatteryConfig& c = m.config();
  return static_cast<uint16_t>(std::lround(packMv / c.divider / c.adcRefMv * c.adcMax));
}

// Sample every 50 ms for ms milliseconds; count state changes.
int run(BatteryMonitor& m, uint32_t& now, uint32_t ms, float packMv) {
  int changes = 0;
  BatteryState last = m.state();
  for (uint32_t i = 0; i < ms; i += 50, now += 50) {
    m.sample(now, adcFor(m, packMv));
    if (m.state() != last) ++changes;
    last = m.state();
  }
  return changes;
}
}  // namespace

int main() {
  {
    BatteryMonitor m;
    // Conversion: 744 counts -> about 7.2 V with the provisional divider.
    assert(std::fabs(m.toMillivolts(744) - 7200.0f) < 10.0f);
    uint32_t now = 0;
    run(m, now, 1500, 7200);
    assert(m.state() == BatteryState::Unknown);  // settling after boot
    run(m, now, 1000, 7200);
    assert(m.state() == BatteryState::Ok);
    // Motor start sag: 0.8 s at 5.5 V, repeated, never declares LOW.
    for (int k = 0; k < 5; ++k) {
      assert(run(m, now, 800, 5500) == 0);
      assert(run(m, now, 3000, 7000) == 0);
    }
    assert(m.state() == BatteryState::Ok);
    // Sustained drop: LOW only after filtering plus the 5 s hold.
    run(m, now, 5000, 6200);
    assert(m.state() == BatteryState::Ok);
    run(m, now, 2000, 6200);
    assert(m.low());
    // Recovery between thresholds (hysteresis band): stays LOW.
    assert(run(m, now, 30000, 6600) == 0 && m.low());
    // Brief recovery above clear: still LOW (10 s hold).
    run(m, now, 4000, 7200);
    assert(m.low());
    // A dip back into the band restarts the clear hold.
    run(m, now, 3000, 6600);
    run(m, now, 9000, 7200);
    assert(m.low());
    run(m, now, 3000, 7200);
    assert(m.state() == BatteryState::Ok);
    // Noise right at the threshold: no flapping.
    int changes = 0;
    for (int k = 0; k < 40; ++k) changes += run(m, now, 500, (k % 2) ? 6300 : 6550);
    assert(changes == 0 && m.state() == BatteryState::Ok);
    // Pack unplugged / USB-only bench: ABSENT, not LOW.
    run(m, now, 3000, 0);
    assert(m.state() == BatteryState::Absent && !m.low());
    run(m, now, 3000, 7200);
    assert(m.state() == BatteryState::Ok);
  }
  {
    // Booting on a flat pack: LOW after settle + hold, never OK in between
    // except as the settled value above lowMv.
    BatteryMonitor m;
    uint32_t now = UINT32_MAX - 3000;  // millis() wrap during the hold
    run(m, now, 6000, 6000);
    assert(m.state() == BatteryState::Unknown || m.state() == BatteryState::Low);
    run(m, now, 2000, 6000);
    assert(m.low());
  }
  {
    // Thresholds and divider are configuration, not code.
    BatteryConfig c;
    c.divider = 2.0f;
    c.lowMv = 4600;
    c.clearMv = 4900;
    BatteryMonitor m(c);
    uint32_t now = 0;
    run(m, now, 3000, 5000);
    assert(m.state() == BatteryState::Ok);
    run(m, now, 8000, 4500);
    assert(m.low());
  }
  std::puts("battery tests passed");
}
