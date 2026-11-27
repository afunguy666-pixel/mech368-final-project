// Internal runtime shared by the host Arduino libraries and the simulation
// harness. Sketches never include this header.
#ifndef FWSIM_RUNTIME_H
#define FWSIM_RUNTIME_H

#include <ArduinoBLE.h>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace fwsim {

constexpr int kMaxPins = 64;
constexpr uint64_t kNever = UINT64_MAX;

// ------------------------------------------------------------ sketches ---
struct SketchEntry {
  std::string name;
  void (*setup)();
  void (*loop)();
};
void registerSketch(const char* name, void (*setup)(), void (*loop)());
const SketchEntry* findSketch(const std::string& name);
std::vector<std::string> sketchNames();
struct SketchRegistrar {
  SketchRegistrar(const char* name, void (*setup)(), void (*loop)()) {
    registerSketch(name, setup, loop);
  }
};

// Thrown when one setup()/loop() call runs for too long in virtual time.
struct StuckSketch : std::runtime_error {
  explicit StuckSketch(const std::string& m) : std::runtime_error(m) {}
};

// ----------------------------------------------------------------- board ---
struct ImuSample {
  float ax = 0, ay = 0, az = 1;      // g
  float gx = 0, gy = 0, gz = 0;      // deg/s
  float mx = 0, my = 20, mz = -40;   // uT
};

class Board;
// Hardware attached to a board's pins (vehicle model, hand model...).
class BoardIO {
 public:
  virtual ~BoardIO() {}
  virtual void sync(Board&) {}  // bring the physical world up to board time
  virtual bool analogInput(Board&, int, double&) { return false; }  // volts
  virtual bool digitalInput(Board&, int, int&) { return false; }
  virtual bool pulseIn(Board&, int, int, uint64_t, uint64_t&) { return false; }
  virtual void outputChanged(Board&, int) {}
  virtual ImuSample imu(Board&) { return ImuSample(); }
  virtual bool gestureDue(Board&) { return false; }
  virtual int takeGesture(Board&) { return -1; }
  virtual int proximity(Board&) { return 255; }
  virtual bool temperature(Board&, double&) { return false; }  // deg C
};

struct PinState {
  int mode = -1;           // -1 = never configured
  int out = 0;             // last digitalWrite level
  bool pwm = false;        // analogWrite active
  double duty = 0.0;       // 0..1 when pwm
  bool driven = false;     // input level driven by the world (e.g. encoder)
  int drivenLevel = 0;
  bool forced = false;     // input level forced by the scenario
  int forcedLevel = 0;
  bool analogForced = false;
  double analogVolts = 0.0;
  // interrupt
  voidFuncPtr isr = nullptr;
  int isrMode = 0;
};

struct PinEvent { uint64_t t; int pin; double value; };  // value = level or duty
struct SerialLine { uint64_t t; std::string text; };

struct BleEvent {
  enum Kind { Connected, Disconnected, Write, Notify, Subscribed } kind;
  uint64_t at;
  int conn;
  std::string uuid;
  std::vector<uint8_t> bytes;
  bool local = false;  // disconnect initiated by this board itself
};

struct BleState {
  bool begun = false;
  std::string localName = "Arduino", deviceName = "Arduino", advertisedUuid;
  std::vector<std::shared_ptr<ServiceState>> services;
  bool connectable = true;
  bool advertising = false, advertiseWanted = false;
  uint64_t advertisingSince = 0;
  bool scanning = false;
  std::string scanUuid, scanName, scanAddress;
  uint64_t scanSince = 0;
  std::vector<int> reported;  // board ids already returned by available()
  BLEDeviceEventHandler handlers[3] = {nullptr, nullptr, nullptr};
  unsigned long timeoutMs = 5000;
  std::deque<BleEvent> inbox;
};

class Board {
 public:
  Board(int id, const std::string& name) : id(id), name(name) {
    for (double& v : lastLogged) v = -1.0;
  }
  const int id;
  const std::string name;
  const SketchEntry* sketch = nullptr;
  BoardIO* io = nullptr;

  uint64_t clockUs = 0;
  uint64_t powerOnUs = 0;
  bool started = false, poweredOff = false;
  uint64_t callStartUs = 0;
  uint64_t maxCallUs = 20000000;  // 20 s of virtual time per setup()/loop()

  // Cost model (virtual time consumed by API calls).
  uint64_t loopCostUs = 1000, timeCallUs = 2, ioCallUs = 5, analogCallUs = 20;
  uint64_t loopJitterUs = 300;  // each loop() also takes 0..jitter extra
  std::mt19937 rng{1};

  PinState pins[kMaxPins];
  bool interruptsEnabled = true;
  std::deque<std::pair<int, uint64_t>> pendingIsr;  // pin, time
  bool inIsr = false;
  uint64_t isrTimeUs = 0;
  bool syncing = false;
  int analogReadBits = 10, analogWriteBits = 8;

  std::deque<char> serialIn;
  bool usbAttached = true;
  unsigned usbWriteLimit = 128;
  int aclCredits = 4;
  bool imuBeginFails = false;  // IMU.begin() returns 0 (set before power-up)
  bool imuFrozen = false;      // no new IMU samples (e.g. I2C lock-up)
  std::string serialPartial;
  std::vector<SerialLine> serialOut;
  std::vector<PinEvent> pinLog;
  std::function<void(const Board&, const std::string&)> onSerialLine;

  BleState ble;

  uint64_t now() const { return inIsr ? isrTimeUs : clockUs; }
  void spend(uint64_t us);
  void sync();
  // World-driven digital input change; fires interrupts.
  void setInputLevel(int pin, int level, uint64_t t);
  void flushInterrupts();
  int inputLevel(int pin);
  double outputFraction(int pin) const;  // 0..1 (PWM duty or digital level)
  double lastLogged[kMaxPins];
  bool warnedPin[kMaxPins] = {};
  uint64_t timeOffsetUs = 0;  // shifts millis()/micros() (rollover tests)
  void warn(const std::string& m);
  std::function<void(const Board&, const std::string&)> onWarning;
  int outputLevel(int pin) const { return pin >= 0 && pin < kMaxPins ? pins[pin].out : 0; }
  void logPin(int pin, double value);
  int levelAt(int pin, uint64_t t) const;  // from pinLog; -1 if never written
  void powerDown();
};

Board* current();
Board& cur();  // throws if no board is executing
struct CurrentBoard {  // RAII: make b the executing board
  explicit CurrentBoard(Board* b);
  ~CurrentBoard();
  Board* previous;
};

// ----------------------------------------------------------------- radio ---
struct RadioConfig {
  uint64_t latencyUs = 10000;        // one-way, ~1 connection interval
  uint64_t jitterUs = 5000;          // uniform 0..jitter added
  uint64_t supervisionTimeoutUs = 2000000;
  uint64_t scanFindUs = 120000;      // advertising -> available()
  uint64_t connectUs = 40000;        // blocking connect()
  uint64_t discoverUs = 120000;      // blocking discoverService()
  uint64_t attRoundTripUs = 30000;   // subscribe()/write with response
  double blockingSpread = 0.5;       // blocking calls take base * (1 +/- spread)
  double lossCentralToPeripheral = 0, lossPeripheralToCentral = 0;
  double corruptCentralToPeripheral = 0, corruptPeripheralToCentral = 0;
  bool blockCentralToPeripheral = false, blockPeripheralToCentral = false;
  bool linkAvailable = true;  // false = out of range / jammed
};

struct RadioPacket {
  enum Fate { Delivered, Lost, Blocked, Corrupted, LinkDown };
  uint64_t sentUs, deliverUs;
  bool fromCentral;
  int central, peripheral;
  std::string uuid;
  std::vector<uint8_t> bytes;  // as sent (before corruption)
  Fate fate;
};

struct Connection {
  int id, central, peripheral;
  bool centralUp = true, peripheralUp = true;  // each side's view
  bool physical = true;
  uint64_t lostAt = kNever;
  uint64_t lastDeliverToCentral = 0, lastDeliverToPeripheral = 0;
  std::vector<std::string> subscribedUuids;
};

class Air {
 public:
  RadioConfig config;
  std::vector<Board*> boards;
  std::deque<Connection> connections;  // deque: stable references
  std::vector<std::weak_ptr<CharState>> remoteChars;
  std::mt19937 rng{368};
  std::function<void(uint64_t, const std::string&)> log;
  std::function<void(const RadioPacket&)> sniff;

  void reset();
  Board* board(int id) const;
  Connection* connection(int id);
  Connection* activeFor(int boardId, bool asCentral);
  bool reachable(const Board& a, const Board& b) const;
  void setLinkAvailable(bool available, uint64_t now);
  void boardPoweredOff(Board& b, uint64_t now);
  void transmit(Connection& c, bool fromCentral, const std::string& uuid,
                const std::vector<uint8_t>& bytes, BleEvent::Kind kind, uint64_t now);
  void post(Board& to, const BleEvent& e);
  void dropLink(Connection& c, uint64_t now, const std::string& why);
  void process(Board& b);  // deliver due events (inside poll)
  void note(uint64_t t, const std::string& m) { if (log) log(t, m); }
  void closeByLocal(Connection& c, Board& self, uint64_t now);
  uint64_t blocking(uint64_t baseUs);  // randomised duration of a blocking call
  int nextConn = 1;
};
Air& air();

// Implementation detail of the BLE classes.
struct CharState {
  std::string uuid;
  uint16_t props = 0;
  int capacity = 0;
  bool fixedLength = false;
  std::vector<uint8_t> value;
  int owner = -1;  // board id
  bool remote = false;
  int conn = -1;               // remote: connection id
  bool writtenFlag = false;    // local
  bool updatedFlag = false;    // remote
  bool remoteSubscribed = false;
  BLECharacteristicEventHandler handlers[BLECharacteristicEventLast] = {};
};
struct ServiceState {
  std::string uuid;
  std::vector<std::shared_ptr<CharState>> chars;
};
struct DeviceState {
  int viewer = -1;   // board holding this handle
  int peer = -1;     // board it refers to
  int conn = -1;     // connection id (when connected/connecting)
  std::vector<std::shared_ptr<ServiceState>> discovered;
};

std::string lower(const std::string& s);
std::string boardAddress(int id);

}  // namespace fwsim
#endif
