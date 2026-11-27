// Host (PC) stand-in for the Arduino core used by the firmware simulator.
// Sketches compiled against this header run on a virtual clock; every call is
// routed to the simulated board that is currently executing. This is NOT the
// real mbed core: the board build in scripts/compile-firmware.sh stays the
// authority on what compiles for the Nano 33 BLE Sense Rev2.
#ifndef FWSIM_ARDUINO_H
#define FWSIM_ARDUINO_H
#define MECH368_TRACE_ACCEPTED

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cmath>
#include <cstdlib>
#include <string>

using std::abs;

typedef uint8_t byte;
typedef bool boolean;
typedef uint16_t word;
typedef int pin_size_t;
typedef void (*voidFuncPtr)(void);

enum PinStatus { LOW = 0, HIGH = 1, CHANGE = 2, FALLING = 3, RISING = 4 };
enum PinMode { INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2, INPUT_PULLDOWN = 3,
               OUTPUT_OPENDRAIN = 4 };
enum { DEC = 10, HEX = 16, OCT = 8, BIN = 2 };
enum { AR_DEFAULT = 0, AR_INTERNAL = 1, AR_VDD = 2 };

// Arduino Nano 33 BLE (mbed core) pin numbering.
constexpr pin_size_t D0 = 0, D1 = 1, D2 = 2, D3 = 3, D4 = 4, D5 = 5, D6 = 6,
                     D7 = 7, D8 = 8, D9 = 9, D10 = 10, D11 = 11, D12 = 12,
                     D13 = 13;
constexpr pin_size_t A0 = 14, A1 = 15, A2 = 16, A3 = 17, A4 = 18, A5 = 19,
                     A6 = 20, A7 = 21;
constexpr pin_size_t LED_BUILTIN = 13;
constexpr pin_size_t LEDR = 22, LEDG = 23, LEDB = 24, LED_PWR = 25;
constexpr int NUM_DIGITAL_PINS = 26;

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define EULER 2.718281828459045235360287471352

#define constrain(amt, low, high) \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define radians(deg) ((deg) * DEG_TO_RAD)
#define degrees(rad) ((rad) * RAD_TO_DEG)
#define sq(x) ((x) * (x))
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))
#define bit(b) (1UL << (b))
#define bitRead(value, b) (((value) >> (b)) & 0x01)
#define bitSet(value, b) ((value) |= (1UL << (b)))
#define bitClear(value, b) ((value) &= ~(1UL << (b)))
#define bitWrite(value, b, v) ((v) ? bitSet(value, b) : bitClear(value, b))
#define F(s) (s)
#define PROGMEM
#define digitalPinToInterrupt(p) (p)

template <class T, class L>
auto min(const T& a, const L& b) -> decltype((b < a) ? b : a) {
  return (b < a) ? b : a;
}
template <class T, class L>
auto max(const T& a, const L& b) -> decltype((b < a) ? b : a) {
  return (a < b) ? b : a;
}

unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void yield();

void pinMode(pin_size_t pin, int mode);
void digitalWrite(pin_size_t pin, int value);
PinStatus digitalRead(pin_size_t pin);
int analogRead(pin_size_t pin);
void analogWrite(pin_size_t pin, int value);
void analogReadResolution(int bits);
void analogWriteResolution(int bits);
void analogReference(int mode);
unsigned long pulseIn(pin_size_t pin, uint8_t state,
                      unsigned long timeoutUs = 1000000UL);
void tone(pin_size_t pin, unsigned int frequency, unsigned long duration = 0);
void noTone(pin_size_t pin);

void attachInterrupt(pin_size_t pin, voidFuncPtr callback, int mode);
void detachInterrupt(pin_size_t pin);
void interrupts();
void noInterrupts();

long random(long howBig);
long random(long howSmall, long howBig);
void randomSeed(unsigned long seed);
long map(long x, long inMin, long inMax, long outMin, long outMax);

// ---------------------------------------------------------------- String ---
class String {
 public:
  String() {}
  String(const char* s) : s_(s ? s : "") {}
  String(const std::string& s) : s_(s) {}
  String(char c) : s_(1, c) {}
  String(unsigned char v, unsigned char base = DEC);
  String(int v, unsigned char base = DEC);
  String(unsigned int v, unsigned char base = DEC);
  String(long v, unsigned char base = DEC);
  String(unsigned long v, unsigned char base = DEC);
  String(long long v, unsigned char base = DEC);
  String(unsigned long long v, unsigned char base = DEC);
  String(float v, unsigned char decimals = 2);
  String(double v, unsigned char decimals = 2);

  unsigned int length() const { return static_cast<unsigned int>(s_.size()); }
  bool isEmpty() const { return s_.empty(); }
  const char* c_str() const { return s_.c_str(); }
  char charAt(unsigned int i) const { return i < s_.size() ? s_[i] : 0; }
  char operator[](unsigned int i) const { return charAt(i); }
  char& operator[](unsigned int i) { return s_[i]; }
  bool reserve(unsigned int n) { s_.reserve(n); return true; }

  String& operator+=(const String& o) { s_ += o.s_; return *this; }
  bool concat(const String& o) { s_ += o.s_; return true; }
  friend String operator+(const String& a, const String& b) {
    return String(a.s_ + b.s_);
  }
  bool operator==(const String& o) const { return s_ == o.s_; }
  bool operator!=(const String& o) const { return s_ != o.s_; }
  bool operator<(const String& o) const { return s_ < o.s_; }
  bool equals(const String& o) const { return s_ == o.s_; }
  bool equalsIgnoreCase(const String& o) const;
  bool startsWith(const String& p) const { return s_.compare(0, p.s_.size(), p.s_) == 0; }
  bool endsWith(const String& p) const {
    return s_.size() >= p.s_.size() &&
           s_.compare(s_.size() - p.s_.size(), p.s_.size(), p.s_) == 0;
  }
  int indexOf(char c, unsigned int from = 0) const;
  int indexOf(const String& s, unsigned int from = 0) const;
  int lastIndexOf(char c) const;
  String substring(unsigned int from) const;
  String substring(unsigned int from, unsigned int to) const;
  void trim();
  void toUpperCase();
  void toLowerCase();
  void replace(const String& from, const String& to);
  void remove(unsigned int index, unsigned int count = 0xffffffffU);
  long toInt() const { return std::strtol(s_.c_str(), nullptr, 10); }
  float toFloat() const { return std::strtof(s_.c_str(), nullptr); }
  double toDouble() const { return std::strtod(s_.c_str(), nullptr); }
  const std::string& str() const { return s_; }

 private:
  std::string s_;
};

// ----------------------------------------------------------- Print/Stream ---
class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t* buffer, size_t size);
  size_t write(const char* s) { return s ? write(reinterpret_cast<const uint8_t*>(s), strlen(s)) : 0; }
  size_t write(const char* buffer, size_t size) {
    return write(reinterpret_cast<const uint8_t*>(buffer), size);
  }
  virtual void flush() {}

  size_t print(const char* s) { return write(s); }
  size_t print(const String& s) { return write(s.c_str()); }
  size_t print(char c) { return write(static_cast<uint8_t>(c)); }
  size_t print(unsigned char v, int base = DEC) { return printNumber(v, base); }
  size_t print(int v, int base = DEC) { return printSigned(v, base); }
  size_t print(unsigned int v, int base = DEC) { return printNumber(v, base); }
  size_t print(long v, int base = DEC) { return printSigned(v, base); }
  size_t print(unsigned long v, int base = DEC) { return printNumber(v, base); }
  size_t print(long long v, int base = DEC) { return printSigned(v, base); }
  size_t print(unsigned long long v, int base = DEC) { return printNumber(v, base); }
  size_t print(double v, int digits = 2);

  size_t println() { return write("\r\n"); }
  template <class T> size_t println(const T& v) { size_t n = print(v); return n + println(); }
  template <class T> size_t println(const T& v, int format) {
    size_t n = print(v, format); return n + println();
  }

 private:
  size_t printNumber(unsigned long long v, int base);
  size_t printSigned(long long v, int base);
};

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  void setTimeout(unsigned long) {}
  String readString();
  String readStringUntil(char terminator);
  long parseInt();
  float parseFloat();
};

class HardwareSerialHost : public Stream {
 public:
  void begin(unsigned long baud, uint16_t config = 0) { (void)baud; (void)config; }
  void end() {}
  explicit operator bool() const { return true; }  // USB always "attached"
  int available() override;
  int read() override;
  int peek() override;
  void send_nb(uint8_t* buffer, uint32_t size, uint32_t* actual);
  size_t write(uint8_t c) override;
  using Print::write;
};
extern HardwareSerialHost Serial;

#endif
