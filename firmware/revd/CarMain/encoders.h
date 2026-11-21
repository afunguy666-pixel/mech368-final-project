#pragma once
#include "config.h"

// 正交编码器，中断计数。正方向 = 车向前。
namespace Encoders {
  void begin();
  void snapshot(int32_t& left, int32_t& right);
}
