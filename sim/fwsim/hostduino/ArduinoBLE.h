// Host stand-in for ArduinoBLE 1.4.x used by the firmware simulator.
// Covers the peripheral/central subset a two-board sketch normally uses. All
// traffic goes through a simulated radio (sim/fwsim/Ble.cpp) with configurable
// latency, loss, corruption, blocking and link loss. Handlers run only inside
// BLE.poll() (and blocking central calls), like the real library.
#ifndef FWSIM_ARDUINO_BLE_H
#define FWSIM_ARDUINO_BLE_H

#include <Arduino.h>
#include <memory>
#include <vector>

namespace fwsim {
struct CharState;
struct ServiceState;
struct DeviceState;
}  // namespace fwsim

enum BLEProperty {
  BLEBroadcast = 0x01,
  BLERead = 0x02,
  BLEWriteWithoutResponse = 0x04,
  BLEWrite = 0x08,
  BLENotify = 0x10,
  BLEIndicate = 0x20,
  BLEAuthSignedWrite = 0x40,
  BLEExtProp = 0x80,
};
enum BLEDeviceEvent { BLEConnected = 0, BLEDisconnected = 1, BLEDiscovered = 2,
                      BLEDeviceLastEvent };
// As in ArduinoBLE, BLERead (0x02) doubles as the "read" characteristic event.
enum BLECharacteristicEvent { BLESubscribed = 0, BLEUnsubscribed = 1,
                              BLEWritten = 3, BLEUpdated = 3,
                              BLECharacteristicEventLast = 4 };

class BLEDevice;
class BLECharacteristic;
typedef void (*BLEDeviceEventHandler)(BLEDevice device);
typedef void (*BLECharacteristicEventHandler)(BLEDevice device,
                                              BLECharacteristic characteristic);

class BLECharacteristic {
 public:
  BLECharacteristic();
  BLECharacteristic(const char* uuid, uint16_t properties, int valueSize,
                    bool fixedLength = false);
  BLECharacteristic(const char* uuid, uint16_t properties, const char* value);
  explicit BLECharacteristic(std::shared_ptr<fwsim::CharState> state) : s_(state) {}

  explicit operator bool() const { return static_cast<bool>(s_); }
  String uuid() const;
  uint8_t properties() const;
  int valueSize() const;
  const uint8_t* value() const;
  int valueLength() const;
  uint8_t operator[](int offset) const;

  int readValue(uint8_t value[], int length);
  int readValue(void* value, int length);
  int readValue(uint8_t& value);
  int readValue(int8_t& value);
  int readValue(uint16_t& value);
  int readValue(int16_t& value);
  int readValue(uint32_t& value);
  int readValue(int32_t& value);

  int writeValue(const uint8_t value[], int length, bool withResponse = true);
  int writeValue(const void* value, int length, bool withResponse = true);
  int writeValue(const char* value, bool withResponse = true);
  int writeValue(uint8_t value, bool withResponse = true);
  int writeValue(int8_t value, bool withResponse = true);
  int writeValue(uint16_t value, bool withResponse = true);
  int writeValue(int16_t value, bool withResponse = true);
  int writeValue(uint32_t value, bool withResponse = true);
  int writeValue(int32_t value, bool withResponse = true);
  int setValue(const uint8_t value[], int length) { return writeValue(value, length); }
  int setValue(const char* value) { return writeValue(value); }
  int broadcast() { return 1; }

  bool written();
  bool subscribed();
  bool valueUpdated();
  void setEventHandler(int event, BLECharacteristicEventHandler handler);

  bool canRead();
  bool read();
  bool canWrite();
  bool canSubscribe();
  bool subscribe();
  bool canUnsubscribe();
  bool unsubscribe();

  const std::shared_ptr<fwsim::CharState>& state() const { return s_; }

 protected:
  std::shared_ptr<fwsim::CharState> s_;
};

class BLEService {
 public:
  BLEService();
  explicit BLEService(const char* uuid);
  explicit BLEService(std::shared_ptr<fwsim::ServiceState> state) : s_(state) {}
  explicit operator bool() const { return static_cast<bool>(s_); }
  String uuid() const;
  void addCharacteristic(BLECharacteristic& characteristic);
  int characteristicCount() const;
  bool hasCharacteristic(const char* uuid) const;
  BLECharacteristic characteristic(int index) const;
  BLECharacteristic characteristic(const char* uuid) const;
  const std::shared_ptr<fwsim::ServiceState>& state() const { return s_; }

 private:
  std::shared_ptr<fwsim::ServiceState> s_;
};

class BLEDevice {
 public:
  BLEDevice() {}
  explicit BLEDevice(std::shared_ptr<fwsim::DeviceState> state) : s_(state) {}
  explicit operator bool() const { return static_cast<bool>(s_); }
  bool operator==(const BLEDevice& o) const;
  bool operator!=(const BLEDevice& o) const { return !(*this == o); }

  void poll(unsigned long timeout = 0);
  bool connected() const;
  bool disconnect();
  String address() const;
  bool hasLocalName() const;
  String localName() const;
  String deviceName();
  bool hasAdvertisedServiceUuid() const;
  String advertisedServiceUuid() const;
  int rssi();

  bool connect();
  bool discoverAttributes();
  bool discoverService(const char* serviceUuid);
  bool hasService(const char* uuid) const;
  int serviceCount() const;
  BLEService service(int index) const;
  BLEService service(const char* uuid) const;
  int characteristicCount() const;
  bool hasCharacteristic(const char* uuid) const;
  BLECharacteristic characteristic(int index) const;
  BLECharacteristic characteristic(const char* uuid) const;

  const std::shared_ptr<fwsim::DeviceState>& state() const { return s_; }

 private:
  std::shared_ptr<fwsim::DeviceState> s_;
};

class BLELocalDevice {
 public:
  int begin();
  void end();
  void poll(unsigned long timeout = 0);
  bool connected() const;
  bool disconnect();
  String address() const;
  int rssi();

  bool setAdvertisedServiceUuid(const char* uuid);
  bool setAdvertisedService(const BLEService& service);
  bool setManufacturerData(const uint8_t[], int) { return true; }
  bool setManufacturerData(uint16_t, const uint8_t[], int) { return true; }
  bool setLocalName(const char* name);
  void setDeviceName(const char* name);
  void setAppearance(uint16_t) {}
  void addService(BLEService& service);
  int advertise();
  void stopAdvertise();

  int scan(bool withDuplicates = false);
  int scanForName(String name, bool withDuplicates = false);
  int scanForUuid(String uuid, bool withDuplicates = false);
  int scanForAddress(String address, bool withDuplicates = false);
  void stopScan();

  BLEDevice central();
  BLEDevice available();
  void setEventHandler(BLEDeviceEvent event, BLEDeviceEventHandler handler);

  void setAdvertisingInterval(uint16_t) {}
  void setConnectionInterval(uint16_t, uint16_t) {}
  void setSupervisionTimeout(uint16_t) {}
  void setConnectable(bool connectable);
  void setTimeout(unsigned long timeoutMs);
  void debug(Stream&) {}
  void noDebug() {}
};
extern BLELocalDevice BLE;

template <typename T>
class BLETypedCharacteristic : public BLECharacteristic {
 public:
  BLETypedCharacteristic(const char* uuid, unsigned int properties)
      : BLECharacteristic(uuid, static_cast<uint16_t>(properties),
                          static_cast<int>(sizeof(T)), true) {}
  int writeValue(T value) {
    return BLECharacteristic::writeValue(reinterpret_cast<const uint8_t*>(&value),
                                         static_cast<int>(sizeof(T)));
  }
  int setValue(T value) { return writeValue(value); }
  T value() {
    T result = T();
    if (BLECharacteristic::valueLength() >= static_cast<int>(sizeof(T)))
      memcpy(&result, BLECharacteristic::value(), sizeof(T));
    return result;
  }
};
typedef BLETypedCharacteristic<bool> BLEBoolCharacteristic;
typedef BLETypedCharacteristic<char> BLECharCharacteristic;
typedef BLETypedCharacteristic<unsigned char> BLEUnsignedCharCharacteristic;
typedef BLETypedCharacteristic<byte> BLEByteCharacteristic;
typedef BLETypedCharacteristic<short> BLEShortCharacteristic;
typedef BLETypedCharacteristic<unsigned short> BLEUnsignedShortCharacteristic;
typedef BLETypedCharacteristic<word> BLEWordCharacteristic;
typedef BLETypedCharacteristic<int> BLEIntCharacteristic;
typedef BLETypedCharacteristic<unsigned int> BLEUnsignedIntCharacteristic;
typedef BLETypedCharacteristic<long> BLELongCharacteristic;
typedef BLETypedCharacteristic<unsigned long> BLEUnsignedLongCharacteristic;
typedef BLETypedCharacteristic<float> BLEFloatCharacteristic;
typedef BLETypedCharacteristic<double> BLEDoubleCharacteristic;

class BLEStringCharacteristic : public BLECharacteristic {
 public:
  BLEStringCharacteristic(const char* uuid, uint16_t properties, int valueSize)
      : BLECharacteristic(uuid, properties, valueSize) {}
  int writeValue(const String& value) { return BLECharacteristic::writeValue(value.c_str()); }
  int setValue(const String& value) { return writeValue(value); }
  String value() {
    return String(std::string(reinterpret_cast<const char*>(BLECharacteristic::value()),
                              static_cast<size_t>(BLECharacteristic::valueLength())));
  }
};

#endif
