// 固件在环模拟：在虚拟开发板上运行未经修改的机器人和遥控器 Arduino 程序，
// 两块板通过模拟的 BLE 无线链路相连，机器人板接着一辆跑在虚拟场地上的虚拟小车。
//
// 场景文件（sim/fwsim/scenarios/*.cpp）这样使用本 API：
//
//   FWSIM_SCENARIO(my_test, "这个场景检查什么") {
//     sim.loadWorld("sim/worlds/course.world");
//     sim.loadVehicle("sim/vehicles/example.vehicle");
//     sim.useSketch(fwsim::Robot, "robot");
//     sim.useSketch(fwsim::Remote, "remote", 300);   // 300 ms 时上电
//     sim.serialInput(fwsim::Remote, 4000, "e");     // 4 s 时输入 'e'
//     sim.runUntil(6000);
//     FWSIM_CHECK(sim, sim.blinking(fwsim::Robot, LED_BUILTIN, 4500, 6000),
//                 "急停锁存期间机器人 LED 闪烁");
//   }
#ifndef FWSIM_SIMULATION_H
#define FWSIM_SIMULATION_H

#include "Model.h"
#include <Mech368Protocol.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace fwsim {

enum BoardId { Robot = 0, Remote = 1 };

struct Sample {
  uint64_t t = 0;
  Pose2 pose;
  double omegaL = 0, omegaR = 0, dutyL = 0, dutyR = 0, battery = 0, lineOffset = 0;
  std::vector<double> line, distance;
  int robotLed = 0, remoteLed = 0, ledR = 1, ledG = 1, ledB = 1;
  bool robotLink = false, remoteLink = false, colliding = false;
  int statusFlags = -1;  // 遥控器最近收到的 Mech368 状态标志
  double temp = 0, tempTrue = 0;  // HS300x 读数模型 / 真实气温
};

// 任一块板在 Serial 上打印的 "MAP x y 温度" 行（温度建图）。
struct MapPoint {
  uint64_t t = 0;
  int board = 0;
  double x = 0, y = 0, temp = 0;  // 程序记录的值
  Pose2 truePose;                 // 打印这一行时机器人的真实位置
};
struct MapScore {
  int points = 0, uniquePoints = 0;  // 唯一点 = 位置不同（精确到 1 cm）
  double pointsPerM2 = 0;            // 每平方米场地的唯一点数
  double cellSize = 0.2;
  int cellsVisited = 0, cellsTotal = 0;
  double coverage = 0;               // 含有记录点的网格单元所占比例
  double meanPosError = 0, maxPosError = 0;  // 记录位置与真实位置之差（米）
  double meanTempError = 0;  // 记录温度与机器人真实所在位置的真实温度之差
  double meanMapError = 0;   // 记录温度与记录位置处真实温度之差
};

struct Event { uint64_t t; std::string source, text; };

struct FrameRecord {
  uint64_t sentUs = 0, deliverUs = 0;
  bool fromRemote = false;  // 指令（遥控器 -> 机器人）还是状态
  RadioPacket::Fate fate = RadioPacket::Delivered;
  bool decoded = false;
  mech368::ble::Frame frame;
  bool delivered() const { return fate == RadioPacket::Delivered || fate == RadioPacket::Corrupted; }
};

struct CheckResult { bool ok; std::string text, where; uint64_t t; };

class Simulation {
 public:
  Simulation();
  ~Simulation();
  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;

  // ---- 配置（在第一次 runUntil 之前） ----
  void loadWorld(const std::string& path);
  void loadVehicle(const std::string& path);
  World& world() { return world_; }
  VehicleConfig& vehicleConfig() { return vehicleConfig_; }
  void useSketch(BoardId board, const std::string& sketch, double powerOnMs = 0);
  RadioConfig& radio() { return air().config; }  // 运行过程中也可以修改
  void seed(uint32_t value) { seed_ = value; }   // 无线链路和传感器噪声的随机种子（默认 368）
  Board& board(BoardId id) { return *boards_[id]; }
  Vehicle& vehicle();
  ScriptedIO& hand() { return hand_; }

  // ---- 脚本激励（时间单位为虚拟 ms） ----
  void at(double ms, std::function<void(Simulation&)> action, const std::string& label = "");
  void serialInput(BoardId board, double ms, const std::string& text);
  void gesture(double ms, int gesture);                      // 机器人上的 APDS
  void tilt(double ms, double rollDeg, double pitchDeg);     // 遥控器手部姿态关键帧
  void setInput(BoardId board, double ms, int pin, int level);
  void setAnalog(BoardId board, double ms, int pin, double volts);
  void setBattery(double ms, double volts);
  void powerOff(BoardId board, double ms);
  void linkLoss(double fromMs, double toMs);                 // 超出范围
  void block(bool remoteToRobot, double fromMs, double toMs);  // 连着但不通

  // ---- 运行 ----
  void runUntil(double ms);
  double nowMs() const { return static_cast<double>(nowUs_) / 1000.0; }
  bool aborted() const { return aborted_; }

  // ---- 观测 ----
  int transitions(BoardId board, int pin, double fromMs, double toMs) const;
  bool blinking(BoardId board, int pin, double fromMs, double toMs) const;
  bool steady(BoardId board, int pin, int level, double fromMs, double toMs) const;
  int levelAt(BoardId board, int pin, double ms) const;  // -1：程序从未驱动过该引脚
  const std::vector<FrameRecord>& frames() const { return frames_; }
  // 机器人在 ms 时刻或之前发出的最后一条已解码状态报文（没有则为 nullptr）。
  const FrameRecord* statusSentBefore(double ms) const;
  // 在 ms 时刻或之前送达机器人的最后一条指令的时间（ms）；没有则为 -1。
  double lastCommandDelivered(double ms) const;
  bool serialContains(BoardId board, const std::string& text, double fromMs = 0) const;
  const std::vector<Sample>& samples() const { return samples_; }
  const std::vector<Event>& events() const { return events_; }
  bool hasEvent(const std::string& textPart, double fromMs = 0) const;
  const std::vector<MapPoint>& mapPoints() const { return mapPoints_; }
  MapScore scoreMap(double cellSize = 0.2) const;
  double maxLineOffset(double fromMs, double toMs) const;

  // ---- 检查 / 报告 ----
  void log(const std::string& source, const std::string& text);
  void check(bool ok, const std::string& text, const char* file, int line);
  const std::vector<CheckResult>& checks() const { return checks_; }
  bool passed() const;
  void writeReport(const std::string& dir, const std::string& name,
                   const std::string& description) const;
  bool verbose = true;

 private:
  struct Action { uint64_t at; int order; std::function<void(Simulation&)> fn; std::string label; };
  void start();
  void runActions(uint64_t upTo);
  void stepBoard(Board& b);
  void sample(uint64_t t);
  void onPacket(const RadioPacket& p);
  void emitEvent(uint64_t t, const std::string& source, const std::string& text);
  uint64_t worldLimit() const;

  World world_;
  VehicleConfig vehicleConfig_;
  std::unique_ptr<Vehicle> vehicle_;
  ScriptedIO hand_;
  std::unique_ptr<Board> boards_[2];
  std::vector<Action> actions_;
  int actionOrder_ = 0;
  bool started_ = false, aborted_ = false;
  uint64_t nowUs_ = 0;
  std::vector<Sample> samples_;
  std::vector<Event> events_;
  std::vector<FrameRecord> frames_;
  std::vector<CheckResult> checks_;
  std::vector<MapPoint> mapPoints_;
  std::deque<std::pair<uint64_t, int>> pendingStatus_;
  int lastStatusFlags_ = -1;
  int lastFate_[2] = {-1, -1};
  uint32_t seed_ = 368;
};

// ---------------------------------------------------------- 场景 ---
struct ScenarioEntry {
  std::string name, description;
  void (*run)(Simulation&);
};
// 在所有 useSketch() 调用中把程序 `from` 替换成 `to`（用于 A/B 对比）。
void substituteSketch(const std::string& from, const std::string& to);
void registerScenario(const char* name, const char* description, void (*run)(Simulation&));
const std::vector<ScenarioEntry>& scenarios();
struct ScenarioRegistrar {
  ScenarioRegistrar(const char* n, const char* d, void (*r)(Simulation&)) { registerScenario(n, d, r); }
};

}  // namespace fwsim

#define FWSIM_CHECK(sim, condition, message) \
  (sim).check(static_cast<bool>(condition), std::string(message) + "  [" #condition "]", __FILE__, __LINE__)

#define FWSIM_SCENARIO(id, description)                                   \
  static void fwsim_scenario_##id(fwsim::Simulation& sim);                 \
  static fwsim::ScenarioRegistrar fwsim_registrar_##id(#id, description,   \
                                                       &fwsim_scenario_##id); \
  static void fwsim_scenario_##id(fwsim::Simulation& sim)

#endif
