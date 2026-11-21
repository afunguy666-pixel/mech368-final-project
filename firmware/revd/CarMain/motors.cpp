#include "motors.h"
#include "sensors.h"
#include <mbed.h>

namespace {
  mbed::PwmOut* pwmL = nullptr;
  mbed::PwmOut* pwmR = nullptr;
  bool isArmed = false;

  // 每个电机只用一个 PWM 脚（IN1），IN2 作方向：
  //   正转：IN2 = 0，IN1 = PWM(u)          → 驱动 / 滑行
  //   反转：IN2 = 1，IN1 = PWM(1 - |u|)    → 驱动 / 刹车
  // 两个方向的衰减模式不同，开环时正反转速度略有差异，开启轮速闭环后可抵消。
  void driveOne(mbed::PwmOut* pwm, uint8_t in2, float u) {
    u = constrain(u, -1.0f, 1.0f);
    if (fabsf(u) < MOTOR_DEADBAND) {
      digitalWrite(in2, LOW);
      pwm->write(0.0f);              // 两脚皆低：滑行
    } else if (u > 0) {
      digitalWrite(in2, LOW);
      pwm->write(u);
    } else {
      digitalWrite(in2, HIGH);
      pwm->write(1.0f + u);
    }
  }
}

void Motors::begin() {
  pinMode(PIN_ARM, OUTPUT);
  digitalWrite(PIN_ARM, LOW);
  pinMode(PIN_L_IN2, OUTPUT);
  pinMode(PIN_R_IN2, OUTPUT);
  digitalWrite(PIN_L_IN2, LOW);
  digitalWrite(PIN_R_IN2, LOW);

  pwmL = new mbed::PwmOut(digitalPinToPinName(PIN_L_IN1));
  pwmR = new mbed::PwmOut(digitalPinToPinName(PIN_R_IN1));
  pwmL->period_us(1000000 / PWM_FREQ_HZ);
  pwmR->period_us(1000000 / PWM_FREQ_HZ);
  pwmL->write(0.0f);
  pwmR->write(0.0f);
  isArmed = false;
}

bool Motors::arm() {
  isArmed = false;
  set(0, 0);
  digitalWrite(PIN_ARM, LOW);
  delayMicroseconds(50);
  digitalWrite(PIN_ARM, HIGH);        // 上升沿：74HC74 锁存 D = 1
  delayMicroseconds(50);
#if HAS_LATCH
  if (!Sensors::latchSet()) {         // 欠压监测仍拉着 CLR：VSW 太低或电池没开
    digitalWrite(PIN_ARM, LOW);
    return false;
  }
#endif
  isArmed = true;
  return true;
}

void Motors::disarm() {
  digitalWrite(PIN_ARM, LOW);
  isArmed = false;
  set(0, 0);
}

bool Motors::armed() { return isArmed; }

void Motors::set(float left, float right) {
  if (!isArmed) left = right = 0;
#if MOTOR_L_INVERT
  left = -left;
#endif
#if MOTOR_R_INVERT
  right = -right;
#endif
  driveOne(pwmL, PIN_L_IN2, left);
  driveOne(pwmR, PIN_R_IN2, right);
}
