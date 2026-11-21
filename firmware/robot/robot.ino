#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Mech368Protocol.h>

namespace wire = mech368::ble;
wire::Receiver receiver;
BLEService controlService(wire::kService);
// Extra byte detects oversized writes even if ArduinoBLE truncates to capacity.
// Variable length preserves short writes so they can also be rejected.
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
  // Connection-scoped discriminator, NOT authentication or a persistent nonce.
  sessionCounter += static_cast<uint32_t>(micros()) | 1U;
  if (sessionCounter == 0) ++sessionCounter;
  receiver.connect(sessionCounter, millis());
  publishStatus(millis());
}

void onDisconnect(BLEDevice central) {
  (void)central;
  linkUp = false;
  receiver.disconnect(millis());
  // ArduinoBLE resumes peripheral advertising after a disconnect.
}

void onCommand(BLEDevice central, BLECharacteristic characteristic) {
  (void)central;
  uint8_t bytes[wire::kFrameSize];
  const int length = characteristic.valueLength();
  if (length != static_cast<int>(sizeof(bytes))) return;
  if (characteristic.readValue(bytes, sizeof(bytes)) != length) return;
  // Handle each write event immediately: a following motion frame cannot
  // overwrite an E-stop before the loop sees it. No BLE I/O inside this callback.
  receiver.receive(bytes, sizeof(bytes), millis());
}

void setup() {
  // TODO: Verify driver, safe electrical defaults and board wiring before
  // implementing any motor- or battery-related I/O.
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
//setup
//test

void loop() {
  receiver.tick(millis());
  if (bleReady) BLE.poll();
  const uint32_t now = millis();
  // TODO: supply a NEW local APDS-9960 gesture edge. No remote unlock exists.
  receiver.tick(now, false);
  if (linkUp && static_cast<uint32_t>(now - lastStatusAt) >= wire::kSendMs) {
    lastStatusAt = now;
    publishStatus(now);
  }
  const bool fault = !bleReady || !linkUp || !receiver.allowed(now);
  digitalWrite(LED_BUILTIN, fault ? ((now / 125U) % 2U ? HIGH : LOW) : HIGH);
  // ALL actual motor outputs remain absent/disabled, even when allowed() is true.
  // Future motor adapter must use receiver.linear(now)/angular(now) behind the
  // same safety gate after wiring, electrical defaults and timing are verified.
}
