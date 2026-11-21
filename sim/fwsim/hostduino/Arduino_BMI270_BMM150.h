// Host stand-in for Arduino_BMI270_BMM150 (Nano 33 BLE Sense Rev2 IMU).
// Units match the real library: acceleration in g, angular rate in deg/s and
// magnetic field in microtesla. Values come from the simulated vehicle (robot
// board) or the scripted hand tilt (remote board). Axis orientation on the
// real board must still be checked on the bench.
#ifndef FWSIM_ARDUINO_BMI270_BMM150_H
#define FWSIM_ARDUINO_BMI270_BMM150_H

#include <Arduino.h>

class BoschSensorClass {
 public:
  int begin();
  void end() {}
  void debug(Stream&) {}
  int readAcceleration(float& x, float& y, float& z);
  int accelerationAvailable();
  float accelerationSampleRate() { return 100.0f; }
  int readGyroscope(float& x, float& y, float& z);
  int gyroscopeAvailable();
  float gyroscopeSampleRate() { return 100.0f; }
  int readMagneticField(float& x, float& y, float& z);
  int magneticFieldAvailable();
  float magneticFieldSampleRate() { return 10.0f; }
};
extern BoschSensorClass IMU;
#define IMU_BMI270_BMM150 IMU

#endif
