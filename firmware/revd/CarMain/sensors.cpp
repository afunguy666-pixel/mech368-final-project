#include "sensors.h"

namespace {
  uint16_t readSrc(uint8_t src) {
#if HAS_MUX
    digitalWrite(PIN_MUX_S0, src & 1);
    digitalWrite(PIN_MUX_S1, (src >> 1) & 1);
    digitalWrite(PIN_MUX_S2, (src >> 2) & 1);
    delayMicroseconds(MUX_SETTLE_US);
    return analogRead(PIN_MUX_Z);
#else
    return analogRead(src);
#endif
  }

  void readAll(uint16_t* line, uint16_t* obs) {
    for (uint8_t i = 0; i < NUM_LINE; i++) line[i] = readSrc(LINE_SRC[i]);
    obs[0] = readSrc(OBS_SRC[0]);
    obs[1] = readSrc(OBS_SRC[1]);
  }

  uint16_t reflect(uint16_t ambient, uint16_t lit) {
#if SENSOR_ACTIVE_LOW
    return ambient > lit ? ambient - lit : 0;
#else
    return lit > ambient ? lit - ambient : 0;
#endif
  }

  bool obsState[2] = {false, false};
  uint8_t obsCount[2] = {0, 0};

  bool debounce(uint8_t i, bool raw) {
    if (raw == obsState[i]) {
      obsCount[i] = 0;
    } else if (++obsCount[i] >= OBS_DEBOUNCE) {
      obsState[i] = raw;
      obsCount[i] = 0;
    }
    return obsState[i];
  }
}

void Sensors::begin() {
  analogReadResolution(ADC_BITS);
#if HAS_MUX
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
#endif
#if HAS_IR_SWITCH
  pinMode(PIN_IREN, OUTPUT);
  digitalWrite(PIN_IREN, LOW);
#endif
}

void Sensors::scan(Frame& f) {
  uint16_t ambL[NUM_LINE], ambO[2], litL[NUM_LINE], litO[2];

#if HAS_IR_SWITCH
  // 发射管关 → 读环境光；开 → 再读；两次相差即为反射信号，可抵消日光和灯光
  digitalWrite(PIN_IREN, LOW);
  delayMicroseconds(IR_SETTLE_US);
  readAll(ambL, ambO);
  digitalWrite(PIN_IREN, HIGH);
  delayMicroseconds(IR_SETTLE_US);
  readAll(litL, litO);
  digitalWrite(PIN_IREN, LOW);
#else
  readAll(litL, litO);
  for (uint8_t i = 0; i < NUM_LINE; i++) ambL[i] = SENSOR_ACTIVE_LOW ? ADC_MAX : 0;
  ambO[0] = ambO[1] = SENSOR_ACTIVE_LOW ? ADC_MAX : 0;
#endif

  for (uint8_t i = 0; i < NUM_LINE; i++) f.line[i] = reflect(ambL[i], litL[i]);

  for (uint8_t i = 0; i < 2; i++) {
#if OBS_MODE == OBS_MODE_DIGITAL_LOW
    f.obsRaw[i] = litO[i];
    bool raw = litO[i] < ADC_MAX / 2;           // 成品模块：有障碍输出低
#else
    f.obsRaw[i] = reflect(ambO[i], litO[i]);
    bool raw = f.obsRaw[i] > OBS_THRESHOLD;
#endif
    bool s = debounce(i, raw);
    if (i == 0) f.obstacleL = s; else f.obstacleR = s;
  }

#if HAS_VBAT
  f.vbat = analogRead(PIN_VBAT) * (VBAT_ADC_REF / ADC_MAX) * VBAT_DIVIDER;
#endif
#if HAS_FAULT
  f.fault = analogRead(PIN_FAULT) < ADC_MAX / 3;
#endif
  f.latch = latchSet();
}

bool Sensors::latchSet() {
#if HAS_LATCH
  return readSrc(MUX_CH_LATCH) > ADC_MAX / 2;
#else
  return true;
#endif
}
