#include <Arduino.h>
#include <USB/PluggableUSBSerial.h>
#include <ArduinoBLE.h>
#include <Mech368Protocol.h>
#include <Mech368Telemetry.h>
#include <utility/HCI.h>

namespace wire = mech368::ble;
enum class LinkPhase { Retry, Scan, Connect, Discover, Subscribe, Active };
LinkPhase phase = LinkPhase::Retry;
BLEDevice robot;
BLECharacteristic commandCharacteristic;
BLECharacteristic statusCharacteristic;
BLECharacteristic telemetryCharacteristic;
uint32_t linkEpoch = 0;
wire::RemoteLink link;
bool bleReady = false;
bool stopRequested = false;
constexpr int kStopButton = D2; // optional normally-open handheld switch to GND
uint32_t retryAt = 0, activeAt = 0, lastSendAt = 0;

void retry(uint32_t now) {
  link.reset();
  phase = LinkPhase::Retry;
  retryAt = now;
  commandCharacteristic = BLECharacteristic();
  statusCharacteristic = BLECharacteristic();
  telemetryCharacteristic = BLECharacteristic();
  if (robot && robot.connected()) robot.disconnect();
  robot = BLEDevice();
}

void setup() {
  pinMode(kStopButton, INPUT_PULLUP);
#ifdef MECH368_BENCH_SILENCE
  pinMode(D3, INPUT_PULLUP);
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200); // No wait for USB attachment.
  bleReady = BLE.begin();
  if (!bleReady) return;
  // Connect/discovery/subscription remain synchronous inside ArduinoBLE.
  // This limits each ATT wait, not the total duration of a discovery operation.
  BLE.setTimeout(100);
}

void loop() {
  const uint32_t beforePoll = millis();
  digitalWrite(LED_BUILTIN, bleReady && link.healthy(beforePoll) ? HIGH :
               ((beforePoll / 125U) % 2U ? HIGH : LOW));
  // Keep requesting a stop across packet loss/reconnect until the robot ACKs
  // that specific stop with its latch set. Only the robot can clear the latch.
  const bool serialStop = Serial.available() > 0 && Serial.read() == 'e';
  const bool newStop = serialStop || digitalRead(kStopButton) == LOW;
  if (newStop) stopRequested = true;
  if (!bleReady) return;
  BLE.poll();
  uint32_t now = millis();
  switch (phase) {
    case LinkPhase::Retry:
      if (static_cast<uint32_t>(now - retryAt) >= 1000) {
        if (BLE.scanForUuid(wire::kService)) phase = LinkPhase::Scan;
        else retryAt = now;
      }
      break;
    case LinkPhase::Scan:
      robot = BLE.available();
      if (robot) { BLE.stopScan(); phase = LinkPhase::Connect; }
      break;
    case LinkPhase::Connect:
      if (robot.connect()) phase = LinkPhase::Discover;
      else retry(millis());
      break;
    case LinkPhase::Discover:
      if (!robot.connected() || !robot.discoverService(wire::kService)) {
        retry(millis()); break;
      }
      commandCharacteristic = robot.characteristic(wire::kCommand);
      statusCharacteristic = robot.characteristic(wire::kStatus);
      telemetryCharacteristic = robot.characteristic(mech368::telemetry::kUuid);
      if (!commandCharacteristic || !(commandCharacteristic.properties() & BLEWriteWithoutResponse) ||
          !statusCharacteristic || !statusCharacteristic.canSubscribe()) {
        retry(millis()); break;
      }
      phase = LinkPhase::Subscribe;
      break;
    case LinkPhase::Subscribe:
      if (!robot.connected() || !statusCharacteristic.subscribe()) {
        retry(millis()); break;
      }
      // Optional subscription occurs before control starts, once per connection.
      if (!telemetryCharacteristic || !telemetryCharacteristic.canSubscribe() ||
          !telemetryCharacteristic.subscribe()) telemetryCharacteristic = BLECharacteristic();
      ++linkEpoch;
      // Do not read a cached GATT value. Wait for a fresh status notification.
      activeAt = millis(); lastSendAt = activeAt;
      phase = LinkPhase::Active;
      break;
    case LinkPhase::Active: {
      if (!robot.connected()) { retry(now); break; }
      bool freshStatus = false;
      bool stopAcknowledged = false;
      if (statusCharacteristic.valueUpdated()) {
        uint8_t bytes[wire::kFrameSize];
        if (statusCharacteristic.valueLength() == static_cast<int>(sizeof(bytes)) &&
            statusCharacteristic.readValue(bytes, sizeof(bytes)) == sizeof(bytes))
          freshStatus = link.status(bytes, sizeof(bytes), millis(), &stopAcknowledged);
      }
      if (stopAcknowledged && !newStop) stopRequested = false;
      now = millis(); // status processing may have advanced the local clock
      // 450 ms without a progressing application ACK signals a fault. Allow
      // startup grace, then reconnect; duplicate status never extends health.
      if (static_cast<uint32_t>(now - activeAt) >= mech368::MotionSafety::kTimeoutMs &&
          !link.healthy(now)) { retry(now); break; }
      // Reply once to each valid status notification (robot sends every 50 ms),
      // so every command carries the newest challenge and its ACK arrives in
      // the next status. A free-running timer drifted into phases where every
      // other command was stale and the link reconnected (docs/SIMULATOR.md).
      if (link.ready() && freshStatus && HCI.availableAclPackets() > 0) {
#ifdef MECH368_BENCH_SILENCE
        // Fault injection on the disconnected-motor bench. Stop has priority.
        if (!stopRequested && digitalRead(D3) == LOW) break;
#endif
        lastSendAt = now;
        uint8_t bytes[wire::kFrameSize];
        if (wire::encode(link.command(stopRequested), bytes, sizeof(bytes)) &&
            commandCharacteristic.writeValue(bytes, sizeof(bytes), false)) link.sent(stopRequested);
        else { retry(millis()); break; }
      }
      if (link.ready() && telemetryCharacteristic && telemetryCharacteristic.valueUpdated()) {
        namespace tel = mech368::telemetry;
        uint8_t frame[tel::kSize], line[tel::kLineSize];
        if (telemetryCharacteristic.valueLength() == static_cast<int>(sizeof(frame)) &&
            telemetryCharacteristic.readValue(frame, sizeof(frame)) == sizeof(frame) &&
            tel::valid(frame, sizeof(frame))) {
          tel::serialLine(linkEpoch, link.session(), frame, line);
          uint32_t sent;
          _SerialUSB.send_nb(line, sizeof(line), &sent); // drop short writes, never retry
        }
      }
      break;
    }
  }
}
