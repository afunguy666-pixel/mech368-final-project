#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Mech368Protocol.h>

namespace wire = mech368::ble;
wire::Receiver receiver;
BLEService controlService(wire::kService);
// 多留 1 字节：即使 ArduinoBLE 把超长写入截断到容量，也能识别出来。
// 使用可变长度，过短的写入也会保留原长度，从而同样可以拒收。
BLECharacteristic commandCharacteristic(wire::kCommand, BLEWriteWithoutResponse,
                                         wire::kFrameSize + 1);
BLECharacteristic statusCharacteristic(wire::kStatus, BLERead | BLENotify,
                                        wire::kFrameSize);
bool bleReady = false;
bool linkUp = false;
uint32_t lastStatusAt = 0;
uint32_t sessionCounter = 0;

void publishStatus(uint32_t now) {
  uint8_t bytes[wire::kFrameSize];
  if (wire::encode(receiver.status(now), bytes, sizeof(bytes)))
    statusCharacteristic.writeValue(bytes, sizeof(bytes));
}

void onConnect(BLEDevice central) {
  (void)central;
  linkUp = true;
  // 按连接区分的标识，不是认证，也不是持久的一次性随机数。
  sessionCounter += static_cast<uint32_t>(micros()) | 1U;
  if (sessionCounter == 0) ++sessionCounter;
  receiver.connect(sessionCounter, millis());
  publishStatus(millis());
}

void onDisconnect(BLEDevice central) {
  (void)central;
  linkUp = false;
  receiver.disconnect(millis());
  // 断开后 ArduinoBLE 会自动恢复外设广播。
}

void onCommand(BLEDevice central, BLECharacteristic characteristic) {
  (void)central;
  uint8_t bytes[wire::kFrameSize];
  const int length = characteristic.valueLength();
  if (length != static_cast<int>(sizeof(bytes))) return;
  if (characteristic.readValue(bytes, sizeof(bytes)) != length) return;
  // 每次写入事件都立即处理：这样后面跟来的运动报文不会在主循环看到急停之前
  // 把它覆盖掉。本回调内不做任何 BLE I/O。
  receiver.receive(bytes, sizeof(bytes), millis());
}

void setup() {
  // TODO：在实现任何与电机或电池相关的 I/O 之前，先核实驱动器、
  // 安全的电气默认状态和板上接线。
  pinMode(LED_BUILTIN, OUTPUT);
  bleReady = BLE.begin();
  if (!bleReady) return;
  BLE.setLocalName("MECH368-Robot");
  BLE.setAdvertisedService(controlService);
  controlService.addCharacteristic(commandCharacteristic);
  controlService.addCharacteristic(statusCharacteristic);
  BLE.addService(controlService);
  BLE.setEventHandler(BLEConnected, onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  commandCharacteristic.setEventHandler(BLEWritten, onCommand);
  BLE.advertise();
}
// setup 结束
// 测试

void loop() {
  receiver.tick(millis());
  if (bleReady) BLE.poll();
  const uint32_t now = millis();
  // TODO：提供一个新的本地 APDS-9960 手势边沿。不存在遥控解锁。
  receiver.tick(now, false);
  if (linkUp && static_cast<uint32_t>(now - lastStatusAt) >= wire::kSendMs) {
    lastStatusAt = now;
    publishStatus(now);
  }
  const bool fault = !bleReady || !linkUp || !receiver.allowed(now);
  digitalWrite(LED_BUILTIN, fault ? ((now / 125U) % 2U ? HIGH : LOW) : HIGH);
  // 即使 allowed() 为 true，所有实际电机输出仍然不存在/处于禁用状态。
  // 将来的电机适配层必须在接线、电气默认状态和时序都核实之后，
  // 在同一个安全闸门之后使用 receiver.linear(now)/angular(now)。
}
