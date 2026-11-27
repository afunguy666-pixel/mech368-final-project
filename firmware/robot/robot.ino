#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>
#include <BatteryMonitor.h>
#include <DriveCalibration.h>
#include <DirectLineTracker.h>
#include <DriveModeSwitch.h>
#include <Mech368Protocol.h>
#include <SlewLimiter.h>
#include "TelemetryPublisher.h"

namespace wire = mech368::ble;
namespace ctl = mech368::control;
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
// Car-side MANUAL limits, applied after the handheld's own limits. Bench
// values, not approved for the track. The protocol bound is wider.
constexpr float kManualMaxLinear = 0.30f;   // m/s
constexpr float kManualMaxAngular = 3.0f;   // rad/s
ctl::DriveModeSwitch modes;
ctl::SlewLimiter manualLinear(1.0f), manualAngular(8.0f);  // per second
uint32_t lastManualAt = 0;
bool faultSeen = false, faultLit = false;

// Low-battery indicator. A7 reads the pack through a divider. Every value
// here is PROVISIONAL until checked with a multimeter on the real pack (see
// docs/GESTURE_BATTERY_HEATMAP.zh-CN.md). The firmware only indicates; it
// never cuts power or motion because of the battery.
constexpr int kBatteryPin = A7;
constexpr int kLowBatteryLed = D1;  // dedicated red "LOW BAT" LED: D1 -> 1 k -> LED -> GND
ctl::BatteryConfig batteryConfig() {
  ctl::BatteryConfig c;
  c.divider = mech368::calibration::batteryDivider;  // Vpack / VA7, shared with telemetry
  c.lowMv = 6400;      // assumed 7.2 V-class pack (6xNiMH or 2S Li-ion)
  c.clearMv = 6800;    // hysteresis: must recover above this to clear
  c.lowHoldMs = 5000;  // filtered value below lowMv this long
  c.clearHoldMs = 10000;
  return c;
}
ctl::BatteryMonitor battery(batteryConfig());
uint32_t lastBatteryAt = 0;
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

ctl::Gesture toGesture(int gesture) {
  switch (gesture) {
    case GESTURE_UP: return ctl::Gesture::Up;
    case GESTURE_DOWN: return ctl::Gesture::Down;
    case GESTURE_LEFT: return ctl::Gesture::Left;
    case GESTURE_RIGHT: return ctl::Gesture::Right;
    default: return ctl::Gesture::None;
  }
}

// Gestures on the car's APDS-9960 choose and start the drive mode. The remote
// E-stop button is not involved. See libraries/Mech368Control/src/DriveModeSwitch.h.
void handleGesture(int gesture, bool wasLatched, uint32_t now) {
  if (gesture == GESTURE_NONE) return;
  faultLit = false;
  ctl::ModeInputs in;
  in.gesture = toGesture(gesture);
  in.latched = wasLatched;
  in.motionPermitted = receiver.allowed(now);
  in.driving = tracker.active();
  in.manualNeutral = receiver.allowed(now) && receiver.linear(now) == 0 && receiver.angular(now) == 0;
  const ctl::ModeAction action = modes.update(in);
  if (*modes.reason()) tracker.log(modes.reason());
  switch (action) {
    case ctl::ModeAction::Stop: tracker.stop(); break;
    case ctl::ModeAction::StartAuto: tracker.start(); break;
    case ctl::ModeAction::StartManual:
      manualLinear.reset();
      manualAngular.reset();
      lastManualAt = now;
      tracker.startManual();
      break;
    case ctl::ModeAction::None: break;
  }
}

// The newest accepted Motion frame's (v, omega). Receiver returns 0 unless
// MotionSafety currently permits motion, so a lost, invalid or repeated
// packet cannot hold a command: the tracker also stops on the same gate.
void feedManual(uint32_t now) {
  using mech368::line::clampf;
  if (!tracker.manual()) {
    manualLinear.reset();
    manualAngular.reset();
    tracker.setManualTwist(mech368::line::Twist());
    return;
  }
  const float dt = static_cast<uint32_t>(now - lastManualAt) / 1000.0f;
  lastManualAt = now;
  const float v = clampf(receiver.linear(now) / 1000.0f, -kManualMaxLinear, kManualMaxLinear);
  const float w = clampf(receiver.angular(now) / 1000.0f, -kManualMaxAngular, kManualMaxAngular);
  tracker.setManualTwist(mech368::line::Twist(manualLinear.step(v, dt), manualAngular.step(w, dt)));
}

// RGB LED (active low) shows the drive mode. LED_BUILTIN stays the BLE
// fault light. Yellow MANUAL driving, green AUTO following, cyan AUTO
// calibration sweep, red navigation stopped on lost line / blocked (until
// the next gesture), magenta MANUAL selected, blue AUTO selected, blue 1 Hz
// blink nothing selected.
void showMode(uint32_t now) {
  if (tracker.navFault()) {
    if (!faultSeen) faultLit = true;
    faultSeen = true;
  } else {
    faultSeen = false;
  }
  bool r = false, g = false, b = false;
  if (tracker.manual()) r = g = true;
  else if (tracker.running()) g = true;
  else if (tracker.calibrating()) g = b = true;
  else if (faultLit) r = true;
  else if (modes.selected() == ctl::DriveMode::Manual) r = b = true;
  else if (modes.selected() == ctl::DriveMode::Auto) b = true;
  else b = (now / 500U) % 2U == 0;
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
}

// Sample A7 every 50 ms. LOW lights the dedicated D1 LED steadily and makes
// the green power LED dip off twice every 2 s. Neither shares a colour,
// position or rhythm with the orange 8 Hz BLE fault blink on LED_BUILTIN.
void updateBattery(uint32_t now) {
  if (static_cast<uint32_t>(now - lastBatteryAt) >= 50) {
    lastBatteryAt = now;
    battery.sample(now, analogRead(kBatteryPin));
  }
  const bool low = battery.low();
  digitalWrite(kLowBatteryLed, low ? HIGH : LOW);
  const uint32_t phase = now % 2000U;
  const bool dip = low && (phase < 150U || (phase >= 300U && phase < 450U));
  digitalWrite(LED_PWR, dip ? LOW : HIGH);
}

uint8_t modeAndBatteryBits() {
  const uint8_t mode = static_cast<uint8_t>(tracker.manual() ? ctl::DriveMode::Manual
                                            : tracker.active() ? ctl::DriveMode::Auto
                                                               : ctl::DriveMode::Idle);
  return static_cast<uint8_t>((mode << 4) | (static_cast<uint8_t>(battery.state()) << 6));
}

void setup() {
#ifdef MECH368_TRACE_ACCEPTED
  pinMode(D0, OUTPUT);
  digitalWrite(D0, LOW);
#endif
  pinMode(kLowBatteryLed, OUTPUT);
  digitalWrite(kLowBatteryLed, LOW);
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, HIGH);
  tracker.ownLeds = false;
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
  handleGesture(gesture, wasLatched, now);
  feedManual(now);
  tracker.update();
  if (linkUp && static_cast<uint32_t>(now - lastStatusAt) >= wire::kSendMs) {
    lastStatusAt = now;
    publishStatus(millis());
  }
  showMode(now);
  updateBattery(now);
  const bool fault = !bleReady || !linkUp || !receiver.allowed(now);
  digitalWrite(LED_BUILTIN, fault ? ((now / 125U) % 2U ? HIGH : LOW) : HIGH);
  if (linkUp) telemetry.update(tracker, receiver, modeAndBatteryBits());
}
