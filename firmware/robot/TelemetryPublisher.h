#ifndef MECH368_TELEMETRY_PUBLISHER_H
#define MECH368_TELEMETRY_PUBLISHER_H
#include <ArduinoBLE.h>
#include <DirectLineTracker.h>
#include <DriveCalibration.h>
#include <Mech368Telemetry.h>
#include <Wire.h>
#include <utility/HCI.h>

namespace tel = mech368::telemetry;

// HS3003 single-shot on the Nano's internal bus. Leave the conversion time
// between loop iterations. No delay, polling loop or I2C retry in control.
class TemperatureSample {
 public:
  void reset() { pending_ = false; }
  void request() {
    Wire1.beginTransmission(0x44);
    Wire1.write(static_cast<uint8_t>(0));
    pending_ = Wire1.endTransmission() == 0;
    at_ = millis();
  }
  int16_t collect(uint8_t& age10ms) {
    age10ms = 255;
    if (!pending_) return tel::kMissing;
    pending_ = false;
    const uint32_t age = millis() - at_;
    if (age < 35 || age > 500) return tel::kMissing;
    if (Wire1.requestFrom(0x44, 4) != 4) return tel::kMissing;
    const int status = Wire1.read() >> 6;
    Wire1.read();
    const int msb = Wire1.read(), lsb = Wire1.read();
    if (status != 0 || msb < 0 || lsb < 0) return tel::kMissing;
    age10ms = static_cast<uint8_t>(age / 10);
    const int raw = ((msb << 8) | lsb) >> 2;
    return static_cast<int16_t>(lroundf(raw * 16500.0f / 16383.0f - 4000.0f));
  }
 private:
  bool pending_ = false;
  uint32_t at_ = 0;
};

class TelemetryPublisher {
 public:
  BLECharacteristic characteristic{tel::kUuid, BLENotify, tel::kSize};
  void reset(uint32_t now) { schedule_.reset(now); temperature_.reset(); }
  void update(const mech368::DirectLineTracker& tracker, const mech368::ble::Receiver& receiver) {
    if (!characteristic.subscribed()) return;
    tel::Kind kind;
    uint16_t seq;
    const uint32_t now = millis();
    if (!schedule_.due(now, kind, seq)) return;
    // One connected central. Keep one controller ACL buffer free for status.
    if (HCI.availableAclPackets() < 2) return;
    const auto state = tracker.navState();
    using mech368::line::NavState;
    uint8_t flags = 0;
    if (tracker.running()) flags |= tel::Running;
    if (tracker.calibrating()) flags |= tel::Calibrating;
    if ((state >= NavState::AvoidBrake && state <= NavState::RejoinPivot) || state == NavState::Blocked)
      flags |= tel::Detour;
    if ((tracker.running() && !tracker.lineReading().onLine) || state == NavState::Lost)
      flags |= tel::OffLine;
    if (tracker.outputsEnabled()) flags |= tel::Driving;
    if (tracker.gyroBias().ready(50)) flags |= tel::BiasReady;
    if (receiver.estopLatched()) flags |= tel::Latched;
    uint8_t bytes[tel::kSize] = {};
    uint8_t* data = bytes + 8;
    uint32_t sampleMs = tracker.sampleMs();
    namespace wire = mech368::ble;
    switch (kind) {
      case tel::Pose: {
        const auto& odom = tracker.odometry();
        wire::put16(data, tel::scaled16(odom.pose().x * 1000, flags));
        wire::put16(data + 2, tel::scaled16(odom.pose().y * 1000, flags));
        wire::put16(data + 4, tel::scaled16(odom.pose().heading * 1000, flags));
        const float mm = odom.distance() * 1000;
        int32_t travel = INT32_MIN;
        if (isfinite(mm) && fabsf(mm) < 2147483520.0f) travel = static_cast<int32_t>(lroundf(mm));
        else flags |= tel::Clipped;
        wire::put32(data + 6, travel);
        break;
      }
      case tel::Infrared: {
        uint16_t raw[4];
        tracker.sampleInfrared(raw);
        for (int i = 0; i < 4; ++i) wire::put16(data + 2 * i, raw[i]);
        if (!tracker.running() && !tracker.calibrating()) sampleMs = millis();
        wire::put16(data + 8, tracker.running() && tracker.lineReading().onLine
                      ? tel::scaled16(tracker.lineReading().position * 1000, flags) : tel::kMissing);
        break;
      }
      case tel::Wheels:
        wire::put32(data, tracker.leftCount());
        wire::put32(data + 4, tracker.rightCount());
        wire::put16(data + 8, tel::scaled16(tracker.gyroBias().bias() * 1000, flags));
        temperature_.request();  // 300 ms until the environment slot
        break;
      case tel::Environment: {
        const uint16_t adc = analogRead(A7);
        wire::put16(data, adc);
        wire::put16(data + 2, static_cast<uint16_t>(lroundf(adc * (3300.0f / 1023) *
                                                            mech368::calibration::batteryDivider)));
        wire::put16(data + 4, temperature_.collect(data[9]));
        sampleMs = millis();
        data[6] = static_cast<uint8_t>(state);
        data[7] = static_cast<uint8_t>(receiver.status(sampleMs, tracker.outputsEnabled()).linear);
        data[8] = static_cast<uint8_t>(tracker.gyroBias().samples());
        break;
      }
    }
    tel::header(bytes, kind, flags, seq, sampleMs);
    tel::finish(bytes);
    characteristic.writeValue(bytes, sizeof(bytes));
  }
 private:
  tel::Schedule schedule_;
  TemperatureSample temperature_;
};
#endif
