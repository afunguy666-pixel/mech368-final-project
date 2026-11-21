// 供电脑端 Arduino 替身库和模拟框架共用的内部运行时。
// 程序代码不会包含这个头文件。
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

// -------------------------------------------------------------- 程序 ---
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

// 单次 setup()/loop() 调用占用的虚拟时间过长时抛出。
struct StuckSketch : std::runtime_error {
  explicit StuckSketch(const std::string& m) : std::runtime_error(m) {}
};

// ------------------------------------------------------------- 开发板 ---
struct ImuSample {
  float ax = 0, ay = 0, az = 1;      // g
  float gx = 0, gy = 0, gz = 0;      // deg/s
  float mx = 0, my = 20, mz = -40;   // uT
};

class Board;
// 接在开发板引脚上的硬件（小车模型、手部模型等）。
class BoardIO {
 public:
  virtual ~BoardIO() {}
  virtual void sync(Board&) {}  // 把物理世界推进到开发板当前时间
  virtual bool analogInput(Board&, int, double&) { return false; }  // 伏特
  virtual bool digitalInput(Board&, int, int&) { return false; }
  virtual bool pulseIn(Board&, int, int, uint64_t, uint64_t&) { return false; }
  virtual void outputChanged(Board&, int) {}
  virtual ImuSample imu(Board&) { return ImuSample(); }
  virtual bool gestureDue(Board&) { return false; }
  virtual int takeGesture(Board&) { return -1; }
  virtual int proximity(Board&) { return 255; }
  virtual bool temperature(Board&, double&) { return false; }  // 摄氏度
};

struct PinState {
  int mode = -1;           // -1 = 从未配置
  int out = 0;             // 最后一次 digitalWrite 的电平
  bool pwm = false;        // analogWrite 生效中
  double duty = 0.0;       // PWM 时为 0..1
  bool driven = false;     // 由物理世界驱动的输入电平（例如编码器）
  int drivenLevel = 0;
  bool forced = false;     // 由场景强制设定的输入电平
  int forcedLevel = 0;
  bool analogForced = false;
  double analogVolts = 0.0;
  // 中断
  voidFuncPtr isr = nullptr;
  int isrMode = 0;
};

struct PinEvent { uint64_t t; int pin; double value; };  // value = 电平或占空比
struct SerialLine { uint64_t t; std::string text; };

struct BleEvent {
  enum Kind { Connected, Disconnected, Write, Notify, Subscribed } kind;
  uint64_t at;
  int conn;
  std::string uuid;
  std::vector<uint8_t> bytes;
  bool local = false;  // 由本板自己发起的断开
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
  std::vector<int> reported;  // 已由 available() 返回过的开发板 id
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
  uint64_t maxCallUs = 20000000;  // 每次 setup()/loop() 最多 20 s 虚拟时间

  // 耗时模型（各 API 调用消耗的虚拟时间）。
  uint64_t loopCostUs = 1000, timeCallUs = 2, ioCallUs = 5, analogCallUs = 20;
  uint64_t loopJitterUs = 300;  // 每次 loop() 额外耗时 0..jitter
  std::mt19937 rng{1};

  PinState pins[kMaxPins];
  bool interruptsEnabled = true;
  std::deque<std::pair<int, uint64_t>> pendingIsr;  // 引脚，时间
  bool inIsr = false;
  uint64_t isrTimeUs = 0;
  bool syncing = false;
  int analogReadBits = 10, analogWriteBits = 8;

  std::deque<char> serialIn;
  std::string serialPartial;
  std::vector<SerialLine> serialOut;
  std::vector<PinEvent> pinLog;
  std::function<void(const Board&, const std::string&)> onSerialLine;

  BleState ble;

  uint64_t now() const { return inIsr ? isrTimeUs : clockUs; }
  void spend(uint64_t us);
  void sync();
  // 由物理世界驱动的数字输入变化；会触发中断。
  void setInputLevel(int pin, int level, uint64_t t);
  void flushInterrupts();
  int inputLevel(int pin);
  double outputFraction(int pin) const;  // 0..1（PWM 占空比或数字电平）
  double lastLogged[kMaxPins];
  bool warnedPin[kMaxPins] = {};
  uint64_t timeOffsetUs = 0;  // 平移 millis()/micros()（用于测试溢出回绕）
  void warn(const std::string& m);
  std::function<void(const Board&, const std::string&)> onWarning;
  int outputLevel(int pin) const { return pin >= 0 && pin < kMaxPins ? pins[pin].out : 0; }
  void logPin(int pin, double value);
  int levelAt(int pin, uint64_t t) const;  // 取自 pinLog；从未写过则为 -1
  void powerDown();
};

Board* current();
Board& cur();  // 没有开发板在执行时抛出异常
struct CurrentBoard {  // RAII：在作用域内让 b 成为当前执行的开发板
  explicit CurrentBoard(Board* b);
  ~CurrentBoard();
  Board* previous;
};

// ----------------------------------------------------------- 无线链路 ---
struct RadioConfig {
  uint64_t latencyUs = 10000;        // 单向，约 1 个连接间隔
  uint64_t jitterUs = 5000;          // 额外加上 0..jitter 的均匀随机值
  uint64_t supervisionTimeoutUs = 2000000;
  uint64_t scanFindUs = 120000;      // 开始广播 -> available() 能发现
  uint64_t connectUs = 40000;        // 阻塞的 connect()
  uint64_t discoverUs = 120000;      // 阻塞的 discoverService()
  uint64_t attRoundTripUs = 30000;   // subscribe()/带响应的写入
  double blockingSpread = 0.5;       // 阻塞调用耗时为 基准值 * (1 ± spread)
  double lossCentralToPeripheral = 0, lossPeripheralToCentral = 0;
  double corruptCentralToPeripheral = 0, corruptPeripheralToCentral = 0;
  bool blockCentralToPeripheral = false, blockPeripheralToCentral = false;
  bool linkAvailable = true;  // false = 超出范围 / 受干扰
};

struct RadioPacket {
  enum Fate { Delivered, Lost, Blocked, Corrupted, LinkDown };
  uint64_t sentUs, deliverUs;
  bool fromCentral;
  int central, peripheral;
  std::string uuid;
  std::vector<uint8_t> bytes;  // 发送时的内容（损坏之前）
  Fate fate;
};

struct Connection {
  int id, central, peripheral;
  bool centralUp = true, peripheralUp = true;  // 各自一方看到的状态
  bool physical = true;
  uint64_t lostAt = kNever;
  uint64_t lastDeliverToCentral = 0, lastDeliverToPeripheral = 0;
  std::vector<std::string> subscribedUuids;
};

class Air {
 public:
  RadioConfig config;
  std::vector<Board*> boards;
  std::deque<Connection> connections;  // 用 deque：引用保持有效
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
  void process(Board& b);  // 投递已到期的事件（在 poll 内）
  void note(uint64_t t, const std::string& m) { if (log) log(t, m); }
  void closeByLocal(Connection& c, Board& self, uint64_t now);
  uint64_t blocking(uint64_t baseUs);  // 阻塞调用的随机化耗时
  int nextConn = 1;
};
Air& air();

// BLE 类的实现细节。
struct CharState {
  std::string uuid;
  uint16_t props = 0;
  int capacity = 0;
  bool fixedLength = false;
  std::vector<uint8_t> value;
  int owner = -1;  // 开发板 id
  bool remote = false;
  int conn = -1;               // 远端特征值：连接 id
  bool writtenFlag = false;    // 本地特征值
  bool updatedFlag = false;    // 远端特征值
  bool remoteSubscribed = false;
  BLECharacteristicEventHandler handlers[BLECharacteristicEventLast] = {};
};
struct ServiceState {
  std::string uuid;
  std::vector<std::shared_ptr<CharState>> chars;
};
struct DeviceState {
  int viewer = -1;   // 持有这个句柄的开发板
  int peer = -1;     // 它所指向的开发板
  int conn = -1;     // 连接 id（已连接/正在连接时）
  std::vector<std::shared_ptr<ServiceState>> discovered;
};

std::string lower(const std::string& s);
std::string boardAddress(int id);

}  // namespace fwsim
#endif
