// Host stand-in for Arduino_APDS9960 (gesture/proximity/colour sensor).
// Gestures and proximity are injected by the scenario script.
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
  int readProximity();  // 0 = close, 255 = far (as the real library)
  void setGestureSensitivity(uint8_t) {}
  void setInterruptPin(int) {}
  bool setLEDBoost(uint8_t) { return true; }
};
extern APDS9960 APDS;

#endif
