// Host stand-in for Arduino_HS300x (Nano 33 BLE Sense Rev2 temperature and
// humidity sensor). On the robot board the reading follows the simulated
// temperature field at the robot's position, with first-order thermal lag,
// offset and noise from the vehicle file. Other boards read the ambient value.
// Each read is assumed to block for ~10 ms (one measurement); measure the
// real library's timing before relying on this.
#ifndef FWSIM_ARDUINO_HS300X_H
#define FWSIM_ARDUINO_HS300X_H

#include <Arduino.h>

enum { FAHRENHEIT, CELSIUS };

class HS300xClass {
 public:
  int begin();
  void end() {}
  float readTemperature(int units = CELSIUS);
  float readHumidity();
};
extern HS300xClass HS300x;

#endif
