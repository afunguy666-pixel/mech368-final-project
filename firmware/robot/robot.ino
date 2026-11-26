#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>
#include <DirectLineTracker.h>
#include <Mech368Protocol.h>
#include "TelemetryPublisher.h"

namespace wire = mech368::ble;
wire::Receiver receiver;
TelemetryPublisher telemetry;
bool motionPermitted() { return receiver.allowed(millis()); }
mech368::DirectLineTracker tracker(motionPermitted);
void leftEncoderIsr() { tracker.leftEdge(); }
void rightEncoderIsr() { tracker.rightEdge(); }
BLEService controlService(wire::kService);
// Extra byte detects oversized writes even if ArduinoBLE truncates to capacity.
// Variable length preserves short writes so they can also be rejected.
BLECharacteristic commandCharacteristic(wire::kCommand, BLEWriteWithoutResponse,
                                         wire::kFrameSize + 1);
BLECharacteristic statusCharacteristic(wire::kStatus, BLERead | BLENotify,
                                        wire::kFrameSize);
bool bleReady = false;
bool linkUp = false;
bool stopSinceGestureCheck = false;
uint32_t lastStatusAt = 0;
uint32_t sessionCounter = 0;
#ifdef MECH368_TRACE_ACCEPTED
bool acceptedEdge = false; // optional logic-analyser marker on the unused D0
#endif

void publishStatus(uint32_t now) {
  if (HCI.availableAclPackets() < 1) return;
  uint8_t bytes[wire::kFrameSize];
  if (wire::encode(receiver.status(now, tracker.outputsEnabled()), bytes, sizeof(bytes)))
    statusCharacteristic.writeValue(bytes, sizeof(bytes));
}

void onConnect(BLEDevice central) {
  (void)central;
  tracker.stop();
  linkUp = true;
  // Connection-scoped discriminator, NOT authentication or a persistent nonce.
  sessionCounter += static_cast<uint32_t>(micros()) | 1U;
  if (sessionCounter == 0) ++sessionCounter;
  receiver.connect(sessionCounter, millis());
  telemetry.reset(millis());
  publishStatus(millis());
}

void onDisconnect(BLEDevice central) {
  (void)central;
  tracker.stop();
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
  const uint32_t now = millis();
  // A newly arrived heartbeat must not hide an expiry during BLE.poll().
  if (!receiver.allowed(now)) tracker.stop();
  wire::Type accepted;
  if (!receiver.receive(bytes, sizeof(bytes), now, &accepted)) return;
  if (accepted == wire::Type::Stop) {
    tracker.stop();
    stopSinceGestureCheck = true;
  }
#ifdef MECH368_TRACE_ACCEPTED
  else if (receiver.allowed(now)) {
    acceptedEdge = !acceptedEdge;
    digitalWrite(D0, acceptedEdge ? HIGH : LOW);
  }
#endif
}

void setup() {
#ifdef MECH368_TRACE_ACCEPTED
  pinMode(D0, OUTPUT);
  digitalWrite(D0, LOW);
#endif
  tracker.begin(leftEncoderIsr, rightEncoderIsr);
  pinMode(LED_BUILTIN, OUTPUT);
  bleReady = BLE.begin();
  if (!bleReady) return;
  BLE.setLocalName("MECH368-Robot");
  BLE.setAdvertisedService(controlService);
  controlService.addCharacteristic(commandCharacteristic);
  controlService.addCharacteristic(statusCharacteristic);
  controlService.addCharacteristic(telemetry.characteristic);
  BLE.addService(controlService);
  BLE.setEventHandler(BLEConnected, onConnect);
  BLE.setEventHandler(BLEDisconnected, onDisconnect);
  commandCharacteristic.setEventHandler(BLEWritten, onCommand);
  BLE.advertise();
}

void loop() {
  if (!motionPermitted()) tracker.stop();
  if (bleReady) BLE.poll();
  if (!motionPermitted()) tracker.stop();
  const int sensedGesture = tracker.readGesture();
  const int gesture = stopSinceGestureCheck ? GESTURE_NONE : sensedGesture;
  stopSinceGestureCheck = false;
  const uint32_t now = millis();
  const bool wasLatched = receiver.estopLatched();
  receiver.tick(now, wasLatched && gesture != GESTURE_NONE);
  if (gesture == GESTURE_DOWN) tracker.stop();
  // Any recovery gesture is consumed. A later UP on a fresh link starts.
  if (gesture == GESTURE_UP && !wasLatched) tracker.start();
  tracker.update();
  if (linkUp && static_cast<uint32_t>(now - lastStatusAt) >= wire::kSendMs) {
    lastStatusAt = now;
    publishStatus(millis());
  }
  const bool fault = !bleReady || !linkUp || !receiver.allowed(now);
  digitalWrite(LED_BUILTIN, fault ? ((now / 125U) % 2U ? HIGH : LOW) : HIGH);
  if (linkUp) telemetry.update(tracker, receiver);
}
