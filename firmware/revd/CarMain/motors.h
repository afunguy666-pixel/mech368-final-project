#pragma once
#include "config.h"

// DRV8833 驱动 + ARM 锁存授权
namespace Motors {
  void begin();                     // 上电第一件事：ARM 拉低，PWM 归零
  bool arm();                       // 产生 ARM 上升沿；RevD 上会回读锁存，未置位返回 false
  void disarm();                    // ARM 拉低：SLEEP 立即变低，电机断开
  bool armed();
  void set(float left, float right);  // -1..1，未授权时输出恒为 0
}
