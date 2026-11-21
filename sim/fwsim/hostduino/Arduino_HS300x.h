// Arduino_HS300x（Nano 33 BLE Sense Rev2 的温湿度传感器）的替身。
// 在机器人板上，读数跟随机器人所在位置的模拟温度场，
// 带有一阶热滞后，以及 vehicle 文件中设定的偏移和噪声。
// 其他开发板读到的是环境温度。
// 假定每次读取阻塞约 10 ms（一次测量）；
// 在依赖这一点之前，请实测真实库的耗时。
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
