#ifndef MECH368_PROTOCOL_H
#define MECH368_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>
#include <MotionSafety.h>

namespace mech368 {
namespace ble {
constexpr size_t kFrameSize = 20;
constexpr uint32_t kSendMs = 50;
constexpr uint32_t kChallengeMs = 200;
constexpr int16_t kMaxLinear = 1000;   // mm/s: protocol bound, NOT approved speed
constexpr int16_t kMaxAngular = 4000;  // mrad/s: protocol bound, NOT approved speed
constexpr char kService[] = "c4130001-3680-4f01-9e10-627e70bca001";
constexpr char kCommand[] = "c4130002-3680-4f01-9e10-627e70bca001";
constexpr char kStatus[] = "c4130003-3680-4f01-9e10-627e70bca001";
enum class Type : uint8_t { Motion = 1, Stop = 2, Status = 3 };
enum Flag : uint16_t { AckValid = 1, EstopLatched = 2, MotionPermitted = 4,
                       OutputsDisabled = 8 };
struct Frame {
  Type type = Type::Motion;
  uint16_t sequence = 0;
  uint32_t session = 0;
  int16_t linear = 0;
  int16_t angular = 0;
  uint32_t challenge = 0;
};
inline bool newer(uint16_t a, uint16_t b) {
  const uint16_t delta = static_cast<uint16_t>(a - b);
  return delta != 0 && delta < 0x8000;
}
inline uint16_t get16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}
inline uint32_t get32(const uint8_t* p) {
  return static_cast<uint32_t>(get16(p)) | (static_cast<uint32_t>(get16(p+2)) << 16);
}
inline int16_t signed16(uint16_t value) {
  return static_cast<int16_t>(value <= 32767 ? static_cast<int32_t>(value)
                                           : static_cast<int32_t>(value) - 65536);
}
inline void put16(uint8_t* p, uint16_t v) { p[0] = v & 255; p[1] = v >> 8; }
inline void put32(uint8_t* p, uint32_t v) { put16(p, v & 65535); put16(p+2, v >> 16); }
inline uint16_t crc16(const uint8_t* p, size_t n) {
  uint16_t crc = 0xffff;
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint16_t>(p[i]) << 8;
    for (uint8_t b = 0; b < 8; ++b)
      crc = static_cast<uint16_t>((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
  }
  return crc;
}
inline bool valid(const Frame& f) {
  if (!f.session || !f.challenge) return false;
  switch (f.type) {
    case Type::Motion: return f.linear >= -kMaxLinear && f.linear <= kMaxLinear &&
                             f.angular >= -kMaxAngular && f.angular <= kMaxAngular;
    case Type::Stop: return f.linear == 0 && f.angular == 0;
    case Type::Status: return f.linear >= 0 && (f.linear & ~15) == 0 && f.angular == 0;
  }
  return false;
}
inline bool encode(const Frame& f, uint8_t* out, size_t n) {
  if (!out || n != kFrameSize || !valid(f)) return false;
  out[0] = 0x36; out[1] = 0x8b; out[2] = 1; out[3] = static_cast<uint8_t>(f.type);
  put16(out+4, f.sequence); put32(out+6, f.session);
  put16(out+10, static_cast<uint16_t>(f.linear));
  put16(out+12, static_cast<uint16_t>(f.angular)); put32(out+14, f.challenge);
  put16(out+18, crc16(out, 18));
  return true;
}
inline bool decode(const uint8_t* p, size_t n, Frame& out) {
  if (!p || n != kFrameSize || p[0] != 0x36 || p[1] != 0x8b || p[2] != 1 ||
      get16(p+18) != crc16(p, 18)) return false;
  Frame f;
  f.type = static_cast<Type>(p[3]); f.sequence = get16(p+4); f.session = get32(p+6);
  f.linear = signed16(get16(p+10)); f.angular = signed16(get16(p+12));
  f.challenge = get32(p+14);
  if (!valid(f)) return false;
  out = f;
  return true;
}

// One instance per robot, shared by BLE callbacks and loop on the same thread.
class Receiver {
 public:
  void connect(uint32_t session, uint32_t now) {
    connected_ = session != 0; session_ = session; challenge_ = 1;
    challengeAt_ = now; hasSequence_ = false; linear_ = angular_ = 0;
    safety_.update(now, false, Packet::None); // invalidate old motion; keep E-stop
  }
  void disconnect(uint32_t now) {
    connected_ = false; linear_ = angular_ = 0;
    safety_.update(now, false, Packet::None);
  }
  void tick(uint32_t now, bool localGesture = false) {
    if (connected_ && static_cast<uint32_t>(now - challengeAt_) >= kChallengeMs)
      rotate(now);
    safety_.update(now, connected_, Packet::None, localGesture);
    if (!safety_.motionAllowed(now)) linear_ = angular_ = 0;
  }
  bool receive(const uint8_t* bytes, size_t n, uint32_t now) {
    tick(now);
    Frame f;
    if (!connected_ || !decode(bytes, n, f) || f.type == Type::Status ||
        f.session != session_ || f.challenge != challenge_ ||
        (hasSequence_ && !newer(f.sequence, sequence_))) return false;
    sequence_ = f.sequence; hasSequence_ = true;
    safety_.update(now, true, f.type == Type::Stop ? Packet::EmergencyStop : Packet::Motion);
    linear_ = safety_.motionAllowed(now) ? f.linear : 0;
    angular_ = safety_.motionAllowed(now) ? f.angular : 0;
    rotate(now); // a challenge may authorize only one accepted command
    return true;
  }
  Frame status(uint32_t now) const {
    Frame f; f.type = Type::Status; f.session = session_; f.challenge = challenge_;
    f.sequence = sequence_; f.linear = OutputsDisabled;
    if (hasSequence_) f.linear |= AckValid;
    if (safety_.emergencyStopLatched()) f.linear |= EstopLatched;
    if (safety_.motionAllowed(now)) f.linear |= MotionPermitted;
    return f;
  }
  bool allowed(uint32_t now) const { return safety_.motionAllowed(now); }
  int16_t linear(uint32_t now) const { return allowed(now) ? linear_ : 0; }
  int16_t angular(uint32_t now) const { return allowed(now) ? angular_ : 0; }
 private:
  void rotate(uint32_t now) { if (++challenge_ == 0) ++challenge_; challengeAt_ = now; }
  MotionSafety safety_;
  uint32_t session_ = 0, challenge_ = 1, challengeAt_ = 0;
  uint16_t sequence_ = 0;
  int16_t linear_ = 0, angular_ = 0;
  bool connected_ = false, hasSequence_ = false;
};

// Remote waits for application acknowledgements, not just a connected radio.
class RemoteLink {
 public:
  void reset() { *this = RemoteLink(); }
  bool status(const uint8_t* bytes, size_t n, uint32_t now) {
    Frame f;
    if (!decode(bytes, n, f) || f.type != Type::Status) return false;
    if (session_ && session_ != f.session) return false;
    if (session_ && f.challenge != challenge_ &&
        static_cast<uint32_t>(f.challenge - challenge_) >= 0x80000000UL) return false;
    session_ = f.session; challenge_ = f.challenge;
    if ((f.linear & AckValid) && sent_ && f.sequence == sentSequence_ &&
        (!ack_ || newer(f.sequence, ackSequence_))) {
      ack_ = true; ackSequence_ = f.sequence; ackAt_ = now;
    }
    return true;
  }
  Frame command(bool stop) const {
    Frame f; f.type = stop ? Type::Stop : Type::Motion;
    f.sequence = nextSequence_; f.session = session_; f.challenge = challenge_;
    // Zero-motion bench heartbeat only. IMU mapping is deliberately not implemented.
    return f;
  }
  void sent() { sent_ = true; sentSequence_ = nextSequence_++; }
  bool ready() const { return session_ != 0; }
  bool healthy(uint32_t now) const {
    return ack_ && static_cast<uint32_t>(now - ackAt_) < MotionSafety::kTimeoutMs;
  }
 private:
  uint32_t session_ = 0, challenge_ = 0, ackAt_ = 0;
  uint16_t nextSequence_ = 0, sentSequence_ = 0, ackSequence_ = 0;
  bool sent_ = false, ack_ = false;
};
} // namespace ble
} // namespace mech368
#endif
