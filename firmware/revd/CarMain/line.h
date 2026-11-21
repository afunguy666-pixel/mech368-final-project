#pragma once
#include "config.h"

// 巡线：逐路最小/最大值校准，加权平均求线的位置
namespace Line {
  void resetCalibration();
  void addCalibrationSample(const uint16_t* refl);
  bool isCalibrated();
  bool position(const uint16_t* refl, float& pos);  // pos：-1 最左 .. +1 最右；没看到线返回 false
  void printCalibration(Stream& s);
}
