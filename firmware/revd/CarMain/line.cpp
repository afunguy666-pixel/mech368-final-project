#include "line.h"

namespace {
  uint16_t lo[NUM_LINE];
  uint16_t hi[NUM_LINE];
  bool calibrated = false;
  const uint16_t MIN_SPAN = 200;  // 每一路黑白差至少这么多计数才算校准成功
}

void Line::resetCalibration() {
  for (uint8_t i = 0; i < NUM_LINE; i++) { lo[i] = ADC_MAX; hi[i] = 0; }
  calibrated = false;
}

void Line::addCalibrationSample(const uint16_t* refl) {
  bool ok = true;
  for (uint8_t i = 0; i < NUM_LINE; i++) {
    if (refl[i] < lo[i]) lo[i] = refl[i];
    if (refl[i] > hi[i]) hi[i] = refl[i];
    if (hi[i] < lo[i] + MIN_SPAN) ok = false;
  }
  calibrated = ok;
}

bool Line::isCalibrated() { return calibrated; }

bool Line::position(const uint16_t* refl, float& pos) {
  float sum = 0, wsum = 0, best = 0;
  for (uint8_t i = 0; i < NUM_LINE; i++) {
    float base = calibrated ? lo[i] : 0.0f;
    float span = calibrated ? float(hi[i] - lo[i]) : float(ADC_MAX);
    float n = constrain((refl[i] - base) / span, 0.0f, 1.0f);   // 0 = 最暗，1 = 最亮
    float d = LINE_IS_DARK ? 1.0f - n : n;                      // 线强度
    if (d > best) best = d;
    d = max(0.0f, d - LINE_NOISE_FLOOR);
    float x = (NUM_LINE > 1) ? (2.0f * i / (NUM_LINE - 1) - 1.0f) : 0.0f;
    sum += d * x;
    wsum += d;
  }
  if (best < LINE_FOUND_THRESH || wsum <= 0) return false;
  pos = sum / wsum;
  return true;
}

void Line::printCalibration(Stream& s) {
  s.print(calibrated ? "line cal OK  " : "line cal INCOMPLETE  ");
  for (uint8_t i = 0; i < NUM_LINE; i++) {
    s.print(lo[i]); s.print('-'); s.print(hi[i]); s.print(' ');
  }
  s.println();
}
