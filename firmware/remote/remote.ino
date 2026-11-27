#include <Arduino.h>
#include <USB/PluggableUSBSerial.h>
#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>
#include <Mech368Protocol.h>
#include <Mech368Telemetry.h>
#include <TiltSteering.h>
#include <utility/HCI.h>
#include <math.h>
#include <stdio.h>

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
constexpr int kZeroButton = D4; // optional normally-open switch to GND: re-zero tilt
uint32_t retryAt = 0, activeAt = 0, lastSendAt = 0;

// Gesture steering. Hold the board flat and still for ~0.5 s after power-up
// to zero it. Tip the front (USB end away from you) down to drive forward,
// the right edge down to turn right. Edit the signs if your grip differs.
// Output is exactly zero without a fresh IMU sample, while zeroing, and
// until the board returns to neutral after any interruption.
mech368::control::TiltConfig tiltConfig() {
  mech368::control::TiltConfig c;
  c.pitchSign = 1.0f;
  c.rollSign = -1.0f;
  c.maxLinear = 0.25f;   // m/s at full tilt, bench value
  c.maxAngular = 2.0f;   // rad/s at full tilt
  return c;
}
mech368::control::TiltSteering tilt(tiltConfig());
mech368::line::Twist tiltCommand;
bool imuReady = false, printTilt = false, zeroWasPressed = false;
uint32_t lastTiltPrint = 0;

void readTilt(uint32_t now) {
  float ax, ay, az;
  if (imuReady && IMU.accelerationAvailable() && IMU.readAcceleration(ax, ay, az))
    tilt.sample(now, ax, ay, az);
  tiltCommand = tilt.update(now);
}

// Red: no IMU or zeroing. Green: zeroed, commanding zero. Blue: commanding
// motion. Magenta: waiting for the board to return to neutral.
void showTilt(uint32_t now) {
  using mech368::control::TiltState;
  const TiltState state = tilt.state(now);
  const bool moving = tiltCommand.v != 0.0f || tiltCommand.omega != 0.0f;
  const bool r = state == TiltState::NoImu || state == TiltState::Zeroing ||
                 state == TiltState::Stale || state == TiltState::WaitNeutral;
  const bool g = state == TiltState::Active && !moving;
  const bool b = (state == TiltState::Active && moving) || state == TiltState::WaitNeutral;
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
}

// Optional bench check of the tilt directions (serial key 't'). One
// non-blocking line every 200 ms; the telemetry recorder ignores it.
void printTiltLine(uint32_t now) {
  if (!printTilt || static_cast<uint32_t>(now - lastTiltPrint) < 200) return;
  lastTiltPrint = now;
  char line[64];
  const int n = snprintf(line, sizeof(line), "TILT %s pitch %d roll %d v %d w %d\n",
                         mech368::control::tiltStateName(tilt.state(now)),
                         static_cast<int>(lroundf(tilt.pitchDeg())), static_cast<int>(lroundf(tilt.rollDeg())),
                         static_cast<int>(lroundf(tiltCommand.v * 1000)),
                         static_cast<int>(lroundf(tiltCommand.omega * 1000)));
  uint32_t sent;
  if (n > 0) _SerialUSB.send_nb(reinterpret_cast<uint8_t*>(line), n < 64 ? n : 63, &sent);
}

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
  pinMode(kZeroButton, INPUT_PULLUP);
  for (int pin : {LEDR, LEDG, LEDB}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
#ifdef MECH368_BENCH_SILENCE
  pinMode(D3, INPUT_PULLUP);
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200); // No wait for USB attachment.
  imuReady = IMU.begin();
  tilt.setImuReady(imuReady);
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
  const int key = Serial.available() > 0 ? Serial.read() : -1;
  const bool serialStop = key == 'e';
  const bool newStop = serialStop || digitalRead(kStopButton) == LOW;
  if (newStop) stopRequested = true;
  const bool zeroPressed = digitalRead(kZeroButton) == LOW;
  if (key == 'z' || (zeroPressed && !zeroWasPressed)) tilt.requestZero();
  zeroWasPressed = zeroPressed;
  if (key == 't') printTilt = !printTilt;
  readTilt(beforePoll);
  showTilt(beforePoll);
  printTiltLine(beforePoll);
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
        // Tilt was sampled at the top of this loop. Stop has priority.
        const int16_t linear = static_cast<int16_t>(lroundf(tiltCommand.v * 1000));
        const int16_t angular = static_cast<int16_t>(lroundf(tiltCommand.omega * 1000));
        if (wire::encode(link.command(stopRequested, linear, angular), bytes, sizeof(bytes)) &&
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
