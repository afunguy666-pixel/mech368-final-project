// Arduino_BMI270_BMM150（Nano 33 BLE Sense Rev2 的 IMU）的替身。
// 单位与真实库一致：加速度为 g，角速度为 deg/s，
// 磁场为微特斯拉。数值来自模拟小车（机器人板）
// 或脚本设定的手部倾斜（遥控器板）。真实开发板上的坐标轴方向
// 仍需在台架上确认。
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
