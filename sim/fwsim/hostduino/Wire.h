#ifndef FWSIM_WIRE_H
#define FWSIM_WIRE_H
#include <stdint.h>
// The tracker uses the internal bus for a split-phase HS3003 conversion.
class HostSensorWire {
 public:
  void beginTransmission(int address) { address_ = address; }
  void write(uint8_t) {}
  int endTransmission();
  int requestFrom(int address, int length);
  int read();
 private:
  int address_ = 0, index_ = 4;
  uint64_t readyAt_ = 0;
  uint8_t data_[4] = {};
};
extern HostSensorWire Wire1;
#endif
