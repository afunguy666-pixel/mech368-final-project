#pragma once
#include <Arduino.h>

// 带积分限幅的简单 PID
struct Pid {
  float kp, ki, kd, iLimit, outLimit;
  float integ = 0, prevErr = 0;
  bool first = true;

  Pid(float p, float i, float d, float iLim, float oLim)
    : kp(p), ki(i), kd(d), iLimit(iLim), outLimit(oLim) {}

  void reset() { integ = 0; prevErr = 0; first = true; }

  float update(float err, float dt) {
    integ = constrain(integ + err * dt, -iLimit, iLimit);
    float deriv = first ? 0.0f : (err - prevErr) / dt;
    first = false;
    prevErr = err;
    return constrain(kp * err + ki * integ + kd * deriv, -outLimit, outLimit);
  }
};
