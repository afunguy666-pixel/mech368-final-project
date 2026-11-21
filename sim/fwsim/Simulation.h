// Firmware-in-the-loop simulation: runs unmodified Arduino sketches for the
// robot and the remote on virtual boards, connected by a simulated BLE radio,
// with the robot board wired to a virtual vehicle on a virtual course.
//
// Scenario files (sim/fwsim/scenarios/*.cpp) use this API:
//
//   FWSIM_SCENARIO(my_test, "what it checks") {
//     sim.loadWorld("sim/worlds/oval.world");
//     sim.loadVehicle("sim/vehicles/example.vehicle");
//     sim.useSketch(fwsim::Robot, "robot");
//     sim.useSketch(fwsim::Remote, "remote", 300);   // powered at 300 ms
//     sim.serialInput(fwsim::Remote, 4000, "e");     // type 'e' at 4 s
//     sim.runUntil(6000);
//     FWSIM_CHECK(sim, sim.blinking(fwsim::Robot, LED_BUILTIN, 4500, 6000),
//                 "robot LED flashes while E-stop is latched");
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
  int statusFlags = -1;  // last Mech368 status flags received by the remote
  double temp = 0, tempTrue = 0;  // HS300x reading model / true air temperature
};

// A "MAP x y tempC" line printed on either board's Serial (temperature mapping).
struct MapPoint {
  uint64_t t = 0;
  int board = 0;
  double x = 0, y = 0, temp = 0;  // what the sketch logged
  Pose2 truePose;                 // where the robot really was when it was printed
};
struct MapScore {
  int points = 0, uniquePoints = 0;  // unique = distinct 1 cm positions
  double pointsPerM2 = 0;            // unique points per m^2 of field
  double cellSize = 0.2;
  int cellsVisited = 0, cellsTotal = 0;
  double coverage = 0;               // fraction of grid cells with a logged point
  double meanPosError = 0, maxPosError = 0;  // logged vs true position (m)
  double meanTempError = 0;  // logged temp vs true temp where the robot really was
  double meanMapError = 0;   // logged temp vs true temp at the logged position
};

struct Event { uint64_t t; std::string source, text; };

struct FrameRecord {
  uint64_t sentUs = 0, deliverUs = 0;
  bool fromRemote = false;  // command (remote -> robot) or status
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

  // ---- configuration (before the first runUntil) ----
  void loadWorld(const std::string& path);
  void loadVehicle(const std::string& path);
  World& world() { return world_; }
  VehicleConfig& vehicleConfig() { return vehicleConfig_; }
  void useSketch(BoardId board, const std::string& sketch, double powerOnMs = 0);
  RadioConfig& radio() { return air().config; }  // may also be changed while running
  void seed(uint32_t value) { seed_ = value; }   // radio + sensor noise (default 368)
  Board& board(BoardId id) { return *boards_[id]; }
  Vehicle& vehicle();
  ScriptedIO& hand() { return hand_; }

  // ---- scripted stimulus (times in virtual ms) ----
  void at(double ms, std::function<void(Simulation&)> action, const std::string& label = "");
  void serialInput(BoardId board, double ms, const std::string& text);
  void gesture(double ms, int gesture);                      // APDS on the robot
  void tilt(double ms, double rollDeg, double pitchDeg);     // remote hand pose keyframe
  void setInput(BoardId board, double ms, int pin, int level);
  void setAnalog(BoardId board, double ms, int pin, double volts);
  void setBattery(double ms, double volts);
  void powerOff(BoardId board, double ms);
  void linkLoss(double fromMs, double toMs);                 // out of range
  void block(bool remoteToRobot, double fromMs, double toMs);  // connected but silent

  // ---- run ----
  void runUntil(double ms);
  double nowMs() const { return static_cast<double>(nowUs_) / 1000.0; }
  bool aborted() const { return aborted_; }

  // ---- observations ----
  int transitions(BoardId board, int pin, double fromMs, double toMs) const;
  bool blinking(BoardId board, int pin, double fromMs, double toMs) const;
  bool steady(BoardId board, int pin, int level, double fromMs, double toMs) const;
  int levelAt(BoardId board, int pin, double ms) const;  // -1: sketch never drove the pin
  const std::vector<FrameRecord>& frames() const { return frames_; }
  // Last decoded status frame the robot sent at or before ms (nullptr if none).
  const FrameRecord* statusSentBefore(double ms) const;
  // Time (ms) of the last command delivered to the robot at or before ms; -1 if none.
  double lastCommandDelivered(double ms) const;
  bool serialContains(BoardId board, const std::string& text, double fromMs = 0) const;
  const std::vector<Sample>& samples() const { return samples_; }
  const std::vector<Event>& events() const { return events_; }
  bool hasEvent(const std::string& textPart, double fromMs = 0) const;
  const std::vector<MapPoint>& mapPoints() const { return mapPoints_; }
  MapScore scoreMap(double cellSize = 0.2) const;
  double maxLineOffset(double fromMs, double toMs) const;

  // ---- checks/report ----
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

// ---------------------------------------------------------- scenarios ---
struct ScenarioEntry {
  std::string name, description;
  void (*run)(Simulation&);
};
// Replace sketch `from` with sketch `to` in every useSketch() call (A/B tests).
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
