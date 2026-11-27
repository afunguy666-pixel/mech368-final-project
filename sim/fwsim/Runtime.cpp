// Host implementation of the Arduino core, Serial, String, IMU and APDS used
// by sketches running inside the firmware simulator.
#include "Runtime.h"
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_HS300x.h>
#include <Wire.h>
#include <utility/HCI.h>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <map>

namespace fwsim {

// -------------------------------------------------------------- registry ---
static std::vector<SketchEntry>& registry() {
  static std::vector<SketchEntry> entries;
  return entries;
}
void registerSketch(const char* name, void (*setup)(), void (*loop)()) {
  registry().push_back(SketchEntry{name, setup, loop});
}
const SketchEntry* findSketch(const std::string& name) {
  for (const auto& e : registry())
    if (e.name == name) return &e;
  return nullptr;
}
std::vector<std::string> sketchNames() {
  std::vector<std::string> names;
  for (const auto& e : registry()) names.push_back(e.name);
  return names;
}

// --------------------------------------------------------- current board ---
static Board* g_current = nullptr;
Board* current() { return g_current; }
Board& cur() {
  if (!g_current)
    throw std::logic_error("Arduino API called outside a simulated board "
                           "(e.g. from a global constructor)");
  return *g_current;
}
CurrentBoard::CurrentBoard(Board* b) : previous(g_current) { g_current = b; }
CurrentBoard::~CurrentBoard() { g_current = previous; }

// ----------------------------------------------------------------- board ---
void Board::spend(uint64_t us) {
  if (inIsr) return;  // ISRs are modelled as instantaneous
  clockUs += us;
  if (clockUs - callStartUs > maxCallUs) {
    char buf[160];
    std::snprintf(buf, sizeof buf,
                  "%s: one %s() call has run for more than %.1f s of virtual time "
                  "(blocking wait that never ends?)",
                  name.c_str(), started ? "loop" : "setup", maxCallUs / 1e6);
    throw StuckSketch(buf);
  }
  sync();
}

void Board::sync() {
  if (!io || syncing || inIsr) return;
  syncing = true;
  try {
    io->sync(*this);
  } catch (...) {
    syncing = false;
    throw;
  }
  syncing = false;
}

void Board::warn(const std::string& m) {
  if (onWarning) onWarning(*this, m);
}

int Board::inputLevel(int pin) {
  if (pin < 0 || pin >= kMaxPins) return 0;
  PinState& p = pins[pin];
  if (p.mode == OUTPUT) return p.out;
  if (p.forced) return p.forcedLevel;
  if (p.driven) return p.drivenLevel;
  int level = 0;
  if (io && io->digitalInput(*this, pin, level)) return level ? 1 : 0;
  if (p.mode == INPUT_PULLUP) return 1;
  return 0;
}

void Board::setInputLevel(int pin, int level, uint64_t t) {
  if (pin < 0 || pin >= kMaxPins) return;
  PinState& p = pins[pin];
  const int old = p.driven ? p.drivenLevel : inputLevel(pin);
  p.driven = true;
  p.drivenLevel = level ? 1 : 0;
  if (old == p.drivenLevel || !p.isr) return;
  const bool rising = p.drivenLevel == 1;
  const bool fire = p.isrMode == CHANGE || (p.isrMode == RISING && rising) ||
                    (p.isrMode == FALLING && !rising) ||
                    (p.isrMode == HIGH && rising) || (p.isrMode == LOW && !rising);
  if (!fire) return;
  if (!interruptsEnabled || inIsr) {
    pendingIsr.push_back(std::make_pair(pin, t));
    return;
  }
  CurrentBoard guard(this);
  inIsr = true;
  isrTimeUs = t;
  try {
    p.isr();
  } catch (...) {
    inIsr = false;
    throw;
  }
  inIsr = false;
  if (!pendingIsr.empty()) flushInterrupts();
}

void Board::flushInterrupts() {
  while (interruptsEnabled && !inIsr && !pendingIsr.empty()) {
    const auto item = pendingIsr.front();
    pendingIsr.pop_front();
    PinState& p = pins[item.first];
    if (!p.isr) continue;
    CurrentBoard guard(this);
    inIsr = true;
    isrTimeUs = std::max(item.second, clockUs);
    p.isr();
    inIsr = false;
  }
}

double Board::outputFraction(int pin) const {
  if (pin < 0 || pin >= kMaxPins) return 0.0;
  const PinState& p = pins[pin];
  if (p.pwm) return p.duty;
  if (p.mode == OUTPUT || p.mode == -1) return p.out;
  return 0.0;
}

void Board::logPin(int pin, double value) {
  if (lastLogged[pin] == value) return;
  lastLogged[pin] = value;
  pinLog.push_back(PinEvent{clockUs, pin, value});
}

int Board::levelAt(int pin, uint64_t t) const {
  for (auto it = pinLog.rbegin(); it != pinLog.rend(); ++it)
    if (it->pin == pin && it->t <= t) return it->value > 0.5 ? 1 : 0;
  return -1;  // never driven by the sketch
}

void Board::powerDown() {
  for (int pin = 0; pin < kMaxPins; ++pin) {
    pins[pin].out = 0;
    pins[pin].pwm = false;
    pins[pin].duty = 0;
    if (lastLogged[pin] >= 0) logPin(pin, 0);
  }
  poweredOff = true;
}

std::string lower(const std::string& s) {
  std::string r = s;
  for (char& c : r) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return r;
}

}  // namespace fwsim

using fwsim::cur;
using fwsim::Board;

// ------------------------------------------------------------------ time ---
unsigned long millis() {
  Board& b = cur();
  b.spend(b.timeCallUs);
  return static_cast<uint32_t>((b.now() + b.timeOffsetUs) / 1000U);
}
unsigned long micros() {
  Board& b = cur();
  b.spend(b.timeCallUs);
  return static_cast<uint32_t>(b.now() + b.timeOffsetUs);
}
void delay(unsigned long ms) {
  Board& b = cur();
  if (b.inIsr) return;
  for (unsigned long i = 0; i < ms; ++i) b.spend(1000);
}
void delayMicroseconds(unsigned int us) { cur().spend(us); }
void yield() { cur().spend(1); }

// ------------------------------------------------------------------ pins ---
static fwsim::PinState& pinRef(Board& b, pin_size_t pin) {
  if (pin < 0 || pin >= fwsim::kMaxPins)
    throw std::out_of_range(b.name + ": invalid pin number " + std::to_string(pin));
  return b.pins[pin];
}

void pinMode(pin_size_t pin, int mode) {
  Board& b = cur();
  b.spend(b.ioCallUs);
  auto& p = pinRef(b, pin);
  p.mode = mode;
  if (mode != OUTPUT) p.pwm = false;
  if (mode == OUTPUT) b.logPin(pin, p.out);
  if (b.io) b.io->outputChanged(b, pin);
}

void digitalWrite(pin_size_t pin, int value) {
  Board& b = cur();
  b.spend(b.ioCallUs);
  auto& p = pinRef(b, pin);
  if (p.mode == -1 && !b.warnedPin[pin]) {
    b.warnedPin[pin] = true;
    b.warn("digitalWrite(" + std::to_string(pin) + ") before pinMode(OUTPUT)");
  }
  p.pwm = false;
  p.out = value ? 1 : 0;
  if (p.mode == OUTPUT || p.mode == -1) b.logPin(pin, p.out);
  if (b.io) b.io->outputChanged(b, pin);
}

PinStatus digitalRead(pin_size_t pin) {
  Board& b = cur();
  b.spend(b.ioCallUs);
  pinRef(b, pin);
  return b.inputLevel(pin) ? HIGH : LOW;
}

int analogRead(pin_size_t pin) {
  Board& b = cur();
  b.spend(b.analogCallUs);
  auto& p = pinRef(b, pin);
  double volts = 0.0;
  if (p.analogForced) volts = p.analogVolts;
  else if (b.io && b.io->analogInput(b, pin, volts)) {}
  else if (p.mode == OUTPUT) volts = p.out ? 3.3 : 0.0;
  const double full = static_cast<double>((1L << b.analogReadBits) - 1);
  const double raw = std::round(volts / 3.3 * full);
  return static_cast<int>(std::max(0.0, std::min(full, raw)));
}

void analogWrite(pin_size_t pin, int value) {
  Board& b = cur();
  b.spend(b.ioCallUs);
  auto& p = pinRef(b, pin);
  const int full = (1 << b.analogWriteBits) - 1;
  const int v = std::max(0, std::min(full, value));
  p.mode = OUTPUT;
  p.pwm = true;
  p.duty = static_cast<double>(v) / full;
  p.out = p.duty >= 0.5 ? 1 : 0;
  b.logPin(pin, p.duty);
  if (b.io) b.io->outputChanged(b, pin);
}

void analogReadResolution(int bits) { cur().analogReadBits = std::max(1, std::min(16, bits)); }
void analogWriteResolution(int bits) { cur().analogWriteBits = std::max(1, std::min(16, bits)); }
void analogReference(int) {}

unsigned long pulseIn(pin_size_t pin, uint8_t state, unsigned long timeoutUs) {
  Board& b = cur();
  b.spend(b.ioCallUs);
  pinRef(b, pin);
  uint64_t duration = 0;
  if (b.io && b.io->pulseIn(b, pin, state, timeoutUs, duration))
    return static_cast<unsigned long>(duration);
  b.spend(timeoutUs);
  return 0;
}

void tone(pin_size_t pin, unsigned int, unsigned long) { analogWrite(pin, 128); }
void noTone(pin_size_t pin) { digitalWrite(pin, LOW); }

void attachInterrupt(pin_size_t pin, voidFuncPtr callback, int mode) {
  Board& b = cur();
  auto& p = pinRef(b, pin);
  if (!p.driven) {  // remember the current level for edge detection
    p.drivenLevel = b.inputLevel(pin);
  }
  p.isr = callback;
  p.isrMode = mode;
}
void detachInterrupt(pin_size_t pin) { pinRef(cur(), pin).isr = nullptr; }
void interrupts() {
  Board& b = cur();
  b.interruptsEnabled = true;
  b.flushInterrupts();
}
void noInterrupts() { cur().interruptsEnabled = false; }

static std::mt19937& arduinoRng() {
  static std::mt19937 rng(368);
  return rng;
}
long random(long howBig) {
  if (howBig <= 0) return 0;
  return static_cast<long>(arduinoRng()() % static_cast<unsigned long>(howBig));
}
long random(long howSmall, long howBig) {
  if (howSmall >= howBig) return howSmall;
  return howSmall + random(howBig - howSmall);
}
void randomSeed(unsigned long seed) { if (seed) arduinoRng().seed(static_cast<uint32_t>(seed)); }
long map(long x, long inMin, long inMax, long outMin, long outMax) {
  if (inMax == inMin) return outMin;
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// ---------------------------------------------------------------- String ---
static std::string numberText(unsigned long long v, int base) {
  if (base < 2 || base > 36) base = 10;
  if (v == 0) return "0";
  std::string s;
  while (v) {
    const int d = static_cast<int>(v % base);
    s.insert(s.begin(), static_cast<char>(d < 10 ? '0' + d : 'A' + d - 10));
    v /= base;
  }
  return s;
}
static std::string signedText(long long v, int base) {
  if (v < 0 && base == 10)
    return "-" + numberText(static_cast<unsigned long long>(-(v + 1)) + 1, base);
  return numberText(static_cast<unsigned long long>(v), base);
}
static std::string floatText(double v, int digits) {
  if (std::isnan(v)) return "nan";
  if (std::isinf(v)) return "inf";
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.*f", std::max(0, digits), v);
  return buf;
}

String::String(unsigned char v, unsigned char base) : s_(numberText(v, base)) {}
String::String(int v, unsigned char base) : s_(signedText(v, base)) {}
String::String(unsigned int v, unsigned char base) : s_(numberText(v, base)) {}
String::String(long v, unsigned char base) : s_(signedText(v, base)) {}
String::String(unsigned long v, unsigned char base) : s_(numberText(v, base)) {}
String::String(long long v, unsigned char base) : s_(signedText(v, base)) {}
String::String(unsigned long long v, unsigned char base) : s_(numberText(v, base)) {}
String::String(float v, unsigned char decimals) : s_(floatText(v, decimals)) {}
String::String(double v, unsigned char decimals) : s_(floatText(v, decimals)) {}

bool String::equalsIgnoreCase(const String& o) const {
  return fwsim::lower(s_) == fwsim::lower(o.s_);
}
int String::indexOf(char c, unsigned int from) const {
  const auto i = s_.find(c, from);
  return i == std::string::npos ? -1 : static_cast<int>(i);
}
int String::indexOf(const String& s, unsigned int from) const {
  const auto i = s_.find(s.s_, from);
  return i == std::string::npos ? -1 : static_cast<int>(i);
}
int String::lastIndexOf(char c) const {
  const auto i = s_.rfind(c);
  return i == std::string::npos ? -1 : static_cast<int>(i);
}
String String::substring(unsigned int from) const {
  return from >= s_.size() ? String() : String(s_.substr(from));
}
String String::substring(unsigned int from, unsigned int to) const {
  if (from > to) std::swap(from, to);
  if (from >= s_.size()) return String();
  return String(s_.substr(from, to - from));
}
void String::trim() {
  const auto first = s_.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) { s_.clear(); return; }
  s_ = s_.substr(first, s_.find_last_not_of(" \t\r\n") - first + 1);
}
void String::toUpperCase() {
  for (char& c : s_) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
}
void String::toLowerCase() { s_ = fwsim::lower(s_); }
void String::replace(const String& from, const String& to) {
  if (from.s_.empty()) return;
  size_t pos = 0;
  while ((pos = s_.find(from.s_, pos)) != std::string::npos) {
    s_.replace(pos, from.s_.size(), to.s_);
    pos += to.s_.size();
  }
}
void String::remove(unsigned int index, unsigned int count) {
  if (index < s_.size()) s_.erase(index, count);
}

// ----------------------------------------------------------- Print/Stream ---
size_t Print::write(const uint8_t* buffer, size_t size) {
  size_t n = 0;
  for (size_t i = 0; i < size; ++i) n += write(buffer[i]);
  return n;
}
size_t Print::printNumber(unsigned long long v, int base) { return write(numberText(v, base).c_str()); }
size_t Print::printSigned(long long v, int base) { return write(signedText(v, base).c_str()); }
size_t Print::print(double v, int digits) { return write(floatText(v, digits).c_str()); }

String Stream::readString() {
  std::string s;
  while (available() > 0) s += static_cast<char>(read());
  return String(s);
}
String Stream::readStringUntil(char terminator) {
  std::string s;
  while (available() > 0) {
    const int c = read();
    if (c == terminator) break;
    s += static_cast<char>(c);
  }
  return String(s);
}
long Stream::parseInt() {
  while (available() > 0 && !std::isdigit(peek()) && peek() != '-') read();
  bool negative = false;
  if (available() > 0 && peek() == '-') { negative = true; read(); }
  long v = 0;
  while (available() > 0 && std::isdigit(peek())) v = v * 10 + (read() - '0');
  return negative ? -v : v;
}
float Stream::parseFloat() {
  std::string s;
  while (available() > 0 && !std::isdigit(peek()) && peek() != '-' && peek() != '.') read();
  while (available() > 0 && (std::isdigit(peek()) || peek() == '-' || peek() == '.'))
    s += static_cast<char>(read());
  return s.empty() ? 0.0f : std::strtof(s.c_str(), nullptr);
}

HardwareSerialHost Serial;
HostHci HCI;
int HostHci::availableAclPackets() const { return cur().aclCredits; }
void HardwareSerialHost::send_nb(uint8_t* buffer, uint32_t size, uint32_t* actual) {
  Board& b = cur();
  *actual = b.usbAttached ? std::min(size, static_cast<uint32_t>(b.usbWriteLimit)) : 0;
  for (uint32_t i = 0; i < *actual; ++i) write(buffer[i]);
}

HostSensorWire Wire1;
int HostSensorWire::endTransmission() {
  Board& b = cur();
  b.spend(50);
  if (address_ != 0x44 || !b.io) return 1;
  double c = 0;
  if (!b.io->temperature(b, c)) return 1;
  const int encoded = static_cast<int>(std::lround((c + 40) * 16383 / 165)) << 2;
  data_[2] = static_cast<uint8_t>(encoded >> 8);
  data_[3] = static_cast<uint8_t>(encoded);
  readyAt_ = b.now() + 35000;
  return 0;
}
int HostSensorWire::requestFrom(int address, int length) {
  cur().spend(100);
  index_ = 0;
  return address == address_ && length == 4 && cur().now() >= readyAt_ ? 4 : 0;
}
int HostSensorWire::read() { return index_ < 4 ? data_[index_++] : -1; }
int HardwareSerialHost::available() {
  Board& b = cur();
  b.spend(b.timeCallUs);
  return static_cast<int>(b.serialIn.size());
}
int HardwareSerialHost::read() {
  Board& b = cur();
  if (b.serialIn.empty()) return -1;
  const int c = static_cast<unsigned char>(b.serialIn.front());
  b.serialIn.pop_front();
  return c;
}
int HardwareSerialHost::peek() {
  Board& b = cur();
  return b.serialIn.empty() ? -1 : static_cast<unsigned char>(b.serialIn.front());
}
size_t HardwareSerialHost::write(uint8_t c) {
  Board& b = cur();
  if (c == '\n') {
    b.serialOut.push_back(fwsim::SerialLine{b.now(), b.serialPartial});
    if (b.onSerialLine) b.onSerialLine(b, b.serialPartial);
    b.serialPartial.clear();
  } else if (c != '\r') {
    b.serialPartial += static_cast<char>(c);
  }
  return 1;
}

// ------------------------------------------------------------------- IMU ---
BoschSensorClass IMU;
namespace {
struct ImuReadState { uint64_t accel = 0, gyro = 0, mag = 0; };
std::map<int, ImuReadState>& imuState() {
  static std::map<int, ImuReadState> s;
  return s;
}
bool newSample(uint64_t& lastIndex, uint64_t nowUs, double rateHz) {
  const uint64_t index = static_cast<uint64_t>(nowUs * rateHz / 1e6) + 1;
  return index > lastIndex;
}
void consume(uint64_t& lastIndex, uint64_t nowUs, double rateHz) {
  lastIndex = static_cast<uint64_t>(nowUs * rateHz / 1e6) + 1;
}
fwsim::ImuSample imuSample(Board& b) {
  b.spend(b.ioCallUs * 20);  // I2C transfer
  return b.io ? b.io->imu(b) : fwsim::ImuSample();
}
}  // namespace

int BoschSensorClass::begin() { cur().spend(2000); return cur().imuBeginFails ? 0 : 1; }
int BoschSensorClass::accelerationAvailable() {
  Board& b = cur(); b.spend(b.ioCallUs);
  if (b.imuFrozen) return 0;
  return newSample(imuState()[b.id].accel, b.now(), accelerationSampleRate());
}
int BoschSensorClass::gyroscopeAvailable() {
  Board& b = cur(); b.spend(b.ioCallUs);
  return newSample(imuState()[b.id].gyro, b.now(), gyroscopeSampleRate());
}
int BoschSensorClass::magneticFieldAvailable() {
  Board& b = cur(); b.spend(b.ioCallUs);
  return newSample(imuState()[b.id].mag, b.now(), magneticFieldSampleRate());
}
int BoschSensorClass::readAcceleration(float& x, float& y, float& z) {
  Board& b = cur();
  const auto s = imuSample(b);
  consume(imuState()[b.id].accel, b.now(), accelerationSampleRate());
  x = s.ax; y = s.ay; z = s.az;
  return 1;
}
int BoschSensorClass::readGyroscope(float& x, float& y, float& z) {
  Board& b = cur();
  const auto s = imuSample(b);
  consume(imuState()[b.id].gyro, b.now(), gyroscopeSampleRate());
  x = s.gx; y = s.gy; z = s.gz;
  return 1;
}
int BoschSensorClass::readMagneticField(float& x, float& y, float& z) {
  Board& b = cur();
  const auto s = imuSample(b);
  consume(imuState()[b.id].mag, b.now(), magneticFieldSampleRate());
  x = s.mx; y = s.my; z = s.mz;
  return 1;
}

// ------------------------------------------------------------------ APDS ---
APDS9960 APDS;
bool APDS9960::begin() { cur().spend(2000); return true; }
int APDS9960::gestureAvailable() {
  Board& b = cur();
  b.spend(b.ioCallUs * 20);
  return b.io && b.io->gestureDue(b) ? 1 : 0;
}
int APDS9960::readGesture() {
  Board& b = cur();
  b.spend(b.ioCallUs * 20);
  return b.io ? b.io->takeGesture(b) : GESTURE_NONE;
}
int APDS9960::readProximity() {
  Board& b = cur();
  b.spend(b.ioCallUs * 20);
  return b.io ? b.io->proximity(b) : 255;
}

// ---------------------------------------------------------------- HS300x ---
HS300xClass HS300x;
int HS300xClass::begin() { cur().spend(2000); return 1; }
float HS300xClass::readTemperature(int units) {
  Board& b = cur();
  b.spend(10000);  // assumed: one blocking measurement
  double c = 22.0;
  if (b.io) b.io->temperature(b, c);
  return static_cast<float>(units == FAHRENHEIT ? c * 9.0 / 5.0 + 32.0 : c);
}
float HS300xClass::readHumidity() {
  cur().spend(10000);
  return 40.0f;
}
