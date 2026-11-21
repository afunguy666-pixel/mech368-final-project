#pragma once
#include "config.h"

namespace Sensors {
  struct Frame {
    uint16_t line[NUM_LINE] = {};  // 反射强度，越大越亮（已扣除环境光）
    uint16_t obsRaw[2] = {};
    bool obstacleL = false;
    bool obstacleR = false;
    float vbat = 0;                // RAW_AA 电压；无此功能时为 0
    bool fault = false;            // DRV8833 nFAULT
    bool latch = true;             // 74HC74 锁存状态；无锁存的板恒为 true
  };

  void begin();
  void scan(Frame& f);   // RevD 上约 1.2 ms：先读环境光，再点亮发射管读一次
  bool latchSet();
}
