#include "encoders.h"

namespace {
  volatile int32_t cntL = 0;
  volatile int32_t cntR = 0;

  // A 超前 B 记为 +1：A 变化时 A≠B 为正；B 变化时 A=B 为正
  void isrLA() { cntL += (digitalRead(PIN_ENC_LA) != digitalRead(PIN_ENC_LB)) ? 1 : -1; }
  void isrRA() { cntR += (digitalRead(PIN_ENC_RA) != digitalRead(PIN_ENC_RB)) ? 1 : -1; }
#if ENC_X4
  void isrLB() { cntL += (digitalRead(PIN_ENC_LA) == digitalRead(PIN_ENC_LB)) ? 1 : -1; }
  void isrRB() { cntR += (digitalRead(PIN_ENC_RA) == digitalRead(PIN_ENC_RB)) ? 1 : -1; }
#endif
}

void Encoders::begin() {
  // RevD 的 R10–R13 默认不装，用内部上拉
  pinMode(PIN_ENC_LA, INPUT_PULLUP);
  pinMode(PIN_ENC_LB, INPUT_PULLUP);
  pinMode(PIN_ENC_RA, INPUT_PULLUP);
  pinMode(PIN_ENC_RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LA), isrLA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RA), isrRA, CHANGE);
#if ENC_X4
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LB), isrLB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RB), isrRB, CHANGE);
#endif
}

void Encoders::snapshot(int32_t& left, int32_t& right) {
  noInterrupts();
  left = cntL;
  right = cntR;
  interrupts();
#if ENC_L_INVERT
  left = -left;
#endif
#if ENC_R_INVERT
  right = -right;
#endif
}
