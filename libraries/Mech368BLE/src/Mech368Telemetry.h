#ifndef MECH368_TELEMETRY_H
#define MECH368_TELEMETRY_H
#include <Mech368Protocol.h>
#include <math.h>

namespace mech368 {
namespace telemetry {
constexpr char kUuid[] = "c4130004-3680-4f01-9e10-627e70bca001";
constexpr size_t kSize = 20;
enum Kind : uint8_t { Pose = 1, Infrared, Wheels, Environment };
enum Flag : uint8_t { Running = 1, Calibrating = 2, Detour = 4, OffLine = 8,
                      Driving = 16, BiasReady = 32, Latched = 64, Clipped = 128 };
constexpr int16_t kMissing = INT16_MIN;

inline int16_t scaled16(float value, uint8_t& flags) {
  if (!isfinite(value) || value < -32767 || value > 32767) {
    flags |= Clipped;
    return kMissing;
  }
  return static_cast<int16_t>(lroundf(value));
}
inline void header(uint8_t* p, Kind kind, uint8_t flags, uint16_t seq, uint32_t ms) {
  p[0] = 0x10 | kind;
  p[1] = flags;
  ble::put16(p + 2, seq);
  ble::put32(p + 4, ms);
}
inline void finish(uint8_t* p) { ble::put16(p + 18, ble::crc16(p, 18)); }
inline bool valid(const uint8_t* p, size_t n) {
  return n == kSize && p && p[0] >= 0x11 && p[0] <= 0x14 &&
         ble::get16(p + 18) == ble::crc16(p, 18);
}

// One slot every 50 ms. Skip missed opportunities and count their sequence
// numbers. Never drain a backlog after reconnection or a delayed loop.
class Schedule {
 public:
  void reset(uint32_t now) {
    last_ = now; slot_ = 0;
    for (auto& s : sequence_) s = 0;
  }
  bool due(uint32_t now, Kind& kind, uint16_t& seq) {
    uint32_t slots = (now - last_) / 50;
    if (!slots) return false;
    last_ += slots * 50;
    const uint32_t cycles = (slots - 1) / 20;
    sequence_[Pose] += cycles * 10;
    for (int k = Infrared; k <= Environment; ++k) sequence_[k] += cycles;
    slots -= cycles * 20;
    while (slots--) {
      const uint8_t s = slot_;
      slot_ = (slot_ + 1) % 20;
      if ((s & 1) == 0) kind = Pose;
      else if (s == 1) kind = Infrared;
      else if (s == 7) kind = Wheels;
      else if (s == 13) kind = Environment;
      else continue;
      seq = sequence_[kind]++;
      if (!slots) return true;
    }
    return false;
  }
 private:
  uint32_t last_ = 0;
  uint16_t sequence_[5] = {};
  uint8_t slot_ = 0;
};

// A complete line fits one full-speed CDC packet. A new '@' resynchronizes
// the reader after a partial write. Both the session tag and data are checked.
constexpr size_t kLineSize = 62;
inline void serialLine(uint32_t epoch, uint32_t session, const uint8_t* frame, uint8_t* out) {
  uint8_t bytes[30];
  ble::put32(bytes, epoch);
  ble::put32(bytes + 4, session);
  for (size_t i = 0; i < kSize; ++i) bytes[i + 8] = frame[i];
  ble::put16(bytes + 28, ble::crc16(bytes, 28));
  const char hex[] = "0123456789ABCDEF";
  out[0] = '@';
  for (size_t i = 0; i < sizeof(bytes); ++i) {
    out[i * 2 + 1] = hex[bytes[i] >> 4];
    out[i * 2 + 2] = hex[bytes[i] & 15];
  }
  out[kLineSize - 1] = '\n';
}
} // namespace telemetry
} // namespace mech368
#endif
