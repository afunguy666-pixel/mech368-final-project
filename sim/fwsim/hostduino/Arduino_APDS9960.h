// Arduino_APDS9960（手势/接近/颜色传感器）的替身。
// 手势和接近度由场景脚本注入。
#ifndef FWSIM_ARDUINO_APDS9960_H
#define FWSIM_ARDUINO_APDS9960_H

#include <Arduino.h>

enum { GESTURE_NONE = -1, GESTURE_UP = 0, GESTURE_DOWN = 1, GESTURE_LEFT = 2,
       GESTURE_RIGHT = 3 };

class APDS9960 {
 public:
  bool begin();
  void end() {}
  int gestureAvailable();
  int readGesture();
  int colorAvailable() { return 1; }
  int readColor(int& r, int& g, int& b) { r = g = b = 0; return 1; }
  int readColor(int& r, int& g, int& b, int& c) { r = g = b = c = 0; return 1; }
  int proximityAvailable() { return 1; }
  int readProximity();  // 0 = 近，255 = 远（与真实库一致）
  void setGestureSensitivity(uint8_t) {}
  void setInterruptPin(int) {}
  bool setLEDBoost(uint8_t) { return true; }
};
extern APDS9960 APDS;

#endif
