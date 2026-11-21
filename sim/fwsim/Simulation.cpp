#include "Simulation.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

namespace fwsim {

namespace {
std::string timeText(uint64_t us) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%8.3f s", static_cast<double>(us) / 1e6);
  return buf;
}
uint64_t toUs(double ms) { return ms <= 0 ? 0 : static_cast<uint64_t>(ms * 1000.0 + 0.5); }
}  // namespace

Simulation::Simulation() {
  boards_[Robot].reset(new Board(Robot, "robot"));
  boards_[Remote].reset(new Board(Remote, "remote"));
}
Simulation::~Simulation() { air().reset(); }

void Simulation::loadWorld(const std::string& path) {
  if (started_) throw std::logic_error("loadWorld() must be called before runUntil()");
  world_ = World::load(path);
}
void Simulation::loadVehicle(const std::string& path) {
  if (started_) throw std::logic_error("loadVehicle() must be called before runUntil()");
  vehicleConfig_ = VehicleConfig::load(path);
}
static std::map<std::string, std::string>& substitutions() {
  static std::map<std::string, std::string> m;
  return m;
}
void substituteSketch(const std::string& from, const std::string& to) { substitutions()[from] = to; }

void Simulation::useSketch(BoardId id, const std::string& requested, double powerOnMs) {
  if (started_) throw std::logic_error("useSketch() must be called before runUntil()");
  const auto sub = substitutions().find(requested);
  const std::string sketch = sub == substitutions().end() ? requested : sub->second;
  const SketchEntry* entry = findSketch(sketch);
  if (!entry) {
    std::string known;
    for (const auto& n : sketchNames()) known += " " + n;
    throw std::runtime_error("no sketch named '" + sketch + "' was compiled in (available:" +
                             known + ")");
  }
  Board& b = board(id);
  b.sketch = entry;
  b.powerOnUs = toUs(powerOnMs);
  b.clockUs = b.powerOnUs;
}
Vehicle& Simulation::vehicle() {
  start();
  return *vehicle_;
}

// ------------------------------------------------------------- scripting ---
void Simulation::at(double ms, std::function<void(Simulation&)> action, const std::string& label) {
  Action a{toUs(ms), actionOrder_++, action, label};
  auto it = std::upper_bound(actions_.begin(), actions_.end(), a,
                             [](const Action& x, const Action& y) {
                               return x.at < y.at || (x.at == y.at && x.order < y.order);
                             });
  actions_.insert(it, a);
}
void Simulation::serialInput(BoardId id, double ms, const std::string& text) {
  at(ms, [id, text](Simulation& s) {
    for (char c : text) s.board(id).serialIn.push_back(c);
  }, "serial input to " + board(id).name + ": \"" + text + "\"");
}
void Simulation::gesture(double ms, int g) {
  static const char* names[] = {"UP", "DOWN", "LEFT", "RIGHT"};
  const uint64_t t = toUs(ms);
  at(0, [t, g](Simulation& s) { s.vehicle().gestures.push_back(std::make_pair(t, g)); });
  at(ms, [](Simulation&) {}, std::string("APDS gesture ") + (g >= 0 && g < 4 ? names[g] : "?") +
                                 " in front of the robot");
}
void Simulation::tilt(double ms, double rollDeg, double pitchDeg) {
  hand_.tilt.push_back(std::make_pair(toUs(ms), Vec2{rollDeg, pitchDeg}));
  std::sort(hand_.tilt.begin(), hand_.tilt.end(),
            [](const std::pair<uint64_t, Vec2>& a, const std::pair<uint64_t, Vec2>& b) {
              return a.first < b.first;
            });
}
void Simulation::setInput(BoardId id, double ms, int pin, int level) {
  at(ms, [id, pin, level](Simulation& s) {
    PinState& p = s.board(id).pins[pin];
    p.forced = true;
    p.forcedLevel = level ? 1 : 0;
  }, board(id).name + " input pin " + std::to_string(pin) + " = " + (level ? "HIGH" : "LOW"));
}
void Simulation::setAnalog(BoardId id, double ms, int pin, double volts) {
  at(ms, [id, pin, volts](Simulation& s) {
    PinState& p = s.board(id).pins[pin];
    p.analogForced = true;
    p.analogVolts = volts;
  }, board(id).name + " analog pin " + std::to_string(pin) + " = " + std::to_string(volts) + " V");
}
void Simulation::setBattery(double ms, double volts) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "battery set to %.2f V", volts);
  at(ms, [volts](Simulation& s) { s.vehicle().batteryVolts = volts; }, buf);
}
void Simulation::powerOff(BoardId id, double ms) {
  at(ms, [id](Simulation& s) {
    Board& b = s.board(id);
    if (b.poweredOff) return;
    b.powerDown();
    air().boardPoweredOff(b, s.nowUs_);
  }, board(id).name + " powered off");
}
void Simulation::linkLoss(double fromMs, double toMs) {
  at(fromMs, [](Simulation& s) { air().setLinkAvailable(false, s.nowUs_); });
  at(toMs, [](Simulation& s) { air().setLinkAvailable(true, s.nowUs_); });
}
void Simulation::block(bool remoteToRobot, double fromMs, double toMs) {
  auto set = [remoteToRobot](bool on) {
    return [remoteToRobot, on](Simulation&) {
      bool remoteIsCentral = true;
      for (const auto& c : air().connections)
        if (c.peripheral == Remote) remoteIsCentral = false;
      const bool c2p = remoteToRobot == remoteIsCentral;
      (c2p ? air().config.blockCentralToPeripheral : air().config.blockPeripheralToCentral) = on;
    };
  };
  const std::string what = remoteToRobot ? "remote -> robot" : "robot -> remote";
  at(fromMs, set(true), "radio: block " + what + " traffic (link stays connected)");
  at(toMs, set(false), "radio: unblock " + what + " traffic");
}

// ------------------------------------------------------------------- run ---
void Simulation::start() {
  if (started_) return;
  started_ = true;
  const RadioConfig radioConfig = air().config;  // keep settings made before start
  air().reset();
  air().config = radioConfig;
  air().rng.seed(seed_);
  air().boards = {boards_[Robot].get(), boards_[Remote].get()};
  air().log = [this](uint64_t t, const std::string& m) { emitEvent(t, "ble", m); };
  air().sniff = [this](const RadioPacket& p) { onPacket(p); };
  vehicle_.reset(new Vehicle(vehicleConfig_, world_));
  vehicle_->board = boards_[Robot].get();
  vehicle_->rng.seed(seed_ + 1);
  boards_[Robot]->rng.seed(seed_ + 2);
  boards_[Remote]->rng.seed(seed_ + 3);
  vehicle_->onEvent = [this](uint64_t t, const std::string& m) { emitEvent(t, "world", m); };
  vehicle_->onSample = [this](uint64_t t) { sample(t); };
  boards_[Robot]->io = vehicle_.get();
  boards_[Remote]->io = &hand_;
  for (auto& b : boards_) {
    b->onSerialLine = [this](const Board& board, const std::string& line) {
      if (verbose) std::cout << "[" << timeText(board.now()) << "] " << board.name << "> " << line << "\n";
      MapPoint p;
      if (std::sscanf(line.c_str(), "MAP %lf %lf %lf", &p.x, &p.y, &p.temp) == 3 ||
          std::sscanf(line.c_str(), "MAP,%lf,%lf,%lf", &p.x, &p.y, &p.temp) == 3) {
        p.t = board.now();
        p.board = board.id;
        p.truePose = vehicle_->pose;
        mapPoints_.push_back(p);
      }
    };
    b->onWarning = [this](const Board& board, const std::string& m) {
      emitEvent(board.now(), board.name, "warning: " + m);
    };
  }
  sample(0);
}

void Simulation::runActions(uint64_t upTo) {
  while (!actions_.empty() && actions_.front().at <= upTo) {
    Action a = actions_.front();
    actions_.erase(actions_.begin());
    nowUs_ = std::max(nowUs_, a.at);
    if (!a.label.empty()) emitEvent(a.at, "script", a.label);
    a.fn(*this);
  }
}

uint64_t Simulation::worldLimit() const {
  const Board& r = *boards_[Robot];
  if (!r.sketch || r.poweredOff) return kNever;
  return r.clockUs;
}

void Simulation::stepBoard(Board& b) {
  CurrentBoard guard(&b);
  b.callStartUs = b.clockUs;
  try {
    b.sync();
    if (!b.started) {
      b.started = true;
      emitEvent(b.clockUs, b.name, "power on: running setup() of sketch '" + b.sketch->name + "'");
      b.sketch->setup();
    } else {
      b.sketch->loop();
    }
    b.clockUs += b.loopCostUs;
    if (b.loopJitterUs) b.clockUs += b.rng() % (b.loopJitterUs + 1);
  } catch (const StuckSketch& e) {
    aborted_ = true;
    emitEvent(b.clockUs, "sim", e.what());
    checks_.push_back(CheckResult{false, e.what(), b.name, b.clockUs});
  } catch (const std::exception& e) {
    aborted_ = true;
    const std::string m = b.name + ": exception escaped the sketch: " + e.what();
    emitEvent(b.clockUs, b.name, m);
    checks_.push_back(CheckResult{false, m, b.name, b.clockUs});
  }
}

void Simulation::runUntil(double ms) {
  start();
  const uint64_t target = toUs(ms);
  while (!aborted_) {
    Board* next = nullptr;
    for (auto& b : boards_)
      if (b->sketch && !b->poweredOff && b->clockUs < target && (!next || b->clockUs < next->clockUs))
        next = b.get();
    const uint64_t t = next ? next->clockUs : target;
    runActions(t);
    if (next && next->poweredOff) continue;
    nowUs_ = std::max(nowUs_, t);
    vehicle_->advanceTo(std::min(t, worldLimit()));
    if (!next) break;
    stepBoard(*next);
  }
  nowUs_ = std::max(nowUs_, target);
  if (!aborted_) vehicle_->advanceTo(std::min(target, worldLimit()));
}

// --------------------------------------------------------------- tracing ---
void Simulation::emitEvent(uint64_t t, const std::string& source, const std::string& text) {
  events_.push_back(Event{t, source, text});
  if (verbose) std::cout << "[" << timeText(t) << "] " << source << ": " << text << "\n";
}

void Simulation::log(const std::string& source, const std::string& text) {
  emitEvent(nowUs_, source, text);
}

void Simulation::onPacket(const RadioPacket& p) {
  FrameRecord r;
  r.sentUs = p.sentUs;
  r.deliverUs = p.deliverUs;
  const int sender = p.fromCentral ? p.central : p.peripheral;
  r.fromRemote = sender == Remote;
  r.fate = p.fate;
  r.decoded = mech368::ble::decode(p.bytes.data(), p.bytes.size(), r.frame);
  frames_.push_back(r);
  if (!r.fromRemote && r.decoded && r.frame.type == mech368::ble::Type::Status && r.delivered() &&
      p.fate == RadioPacket::Delivered)
    pendingStatus_.push_back(std::make_pair(p.deliverUs, static_cast<int>(r.frame.linear)));
  // Report only changes of packet fate per direction, not every packet.
  static const char* fateNames[] = {"delivered", "lost", "blocked", "corrupted", "dropped (link down)"};
  const int dir = r.fromRemote ? 0 : 1;
  if (lastFate_[dir] != static_cast<int>(p.fate)) {
    const bool first = lastFate_[dir] < 0;
    lastFate_[dir] = static_cast<int>(p.fate);
    if (!first || p.fate != RadioPacket::Delivered)
      emitEvent(p.sentUs, "radio", std::string(r.fromRemote ? "remote -> robot" : "robot -> remote") +
                                       " packets: " + fateNames[p.fate]);
  }
}

void Simulation::sample(uint64_t t) {
  while (!pendingStatus_.empty() && pendingStatus_.front().first <= t) {
    lastStatusFlags_ = pendingStatus_.front().second;
    pendingStatus_.pop_front();
  }
  const Vehicle& v = *vehicle_;
  const Board& robot = *boards_[Robot];
  const Board& remote = *boards_[Remote];
  Sample s;
  s.t = t;
  s.pose = v.pose;
  s.omegaL = v.omegaL;
  s.omegaR = v.omegaR;
  s.dutyL = v.dutyL;
  s.dutyR = v.dutyR;
  s.battery = v.batteryVolts;
  s.lineOffset = world_.lines.empty() ? 0.0 : v.lineOffset();
  for (size_t i = 0; i < v.cfg.lineSensors.size(); ++i) s.line.push_back(v.lineSensorCoverage(i));
  for (size_t i = 0; i < v.cfg.distance.size(); ++i) s.distance.push_back(v.distanceReading(i));
  s.robotLed = robot.outputLevel(LED_BUILTIN);
  s.remoteLed = remote.outputLevel(LED_BUILTIN);
  auto rgb = [&](int pin) { return robot.pins[pin].mode == OUTPUT ? robot.outputLevel(pin) : 1; };
  s.ledR = rgb(LEDR);
  s.ledG = rgb(LEDG);
  s.ledB = rgb(LEDB);
  s.robotLink = air().activeFor(Robot, false) || air().activeFor(Robot, true);
  s.remoteLink = air().activeFor(Remote, true) || air().activeFor(Remote, false);
  s.colliding = v.colliding;
  s.statusFlags = s.remoteLink ? lastStatusFlags_ : -1;
  s.temp = v.sensedTemp + v.cfg.tempOffset;
  s.tempTrue = world_.temperature(Vec2{v.pose.x, v.pose.y});
  samples_.push_back(s);
}

// ---------------------------------------------------------- observations ---
int Simulation::transitions(BoardId id, int pin, double fromMs, double toMs) const {
  const uint64_t from = toUs(fromMs), to = toUs(toMs);
  int n = 0;
  int level = levelAt(id, pin, fromMs);
  for (const auto& e : boards_[id]->pinLog) {
    if (e.pin != pin || e.t <= from || e.t > to) continue;
    const int l = e.value > 0.5 ? 1 : 0;
    if (level >= 0 && l != level) ++n;
    level = l;
  }
  return n;
}
bool Simulation::blinking(BoardId id, int pin, double fromMs, double toMs) const {
  return transitions(id, pin, fromMs, toMs) >= 2;
}
bool Simulation::steady(BoardId id, int pin, int level, double fromMs, double toMs) const {
  return levelAt(id, pin, fromMs) == level && transitions(id, pin, fromMs, toMs) == 0;
}
int Simulation::levelAt(BoardId id, int pin, double ms) const {
  return boards_[id]->levelAt(pin, toUs(ms));
}
const FrameRecord* Simulation::statusSentBefore(double ms) const {
  const uint64_t t = toUs(ms);
  for (auto it = frames_.rbegin(); it != frames_.rend(); ++it)
    if (!it->fromRemote && it->decoded && it->frame.type == mech368::ble::Type::Status && it->sentUs <= t)
      return &*it;
  return nullptr;
}
double Simulation::lastCommandDelivered(double ms) const {
  const uint64_t t = toUs(ms);
  double best = -1;
  for (const auto& f : frames_)
    if (f.fromRemote && f.fate == RadioPacket::Delivered && f.deliverUs <= t)
      best = std::max(best, f.deliverUs / 1000.0);
  return best;
}
bool Simulation::serialContains(BoardId id, const std::string& text, double fromMs) const {
  for (const auto& l : boards_[id]->serialOut)
    if (l.t >= toUs(fromMs) && l.text.find(text) != std::string::npos) return true;
  return false;
}
bool Simulation::hasEvent(const std::string& part, double fromMs) const {
  for (const auto& e : events_)
    if (e.t >= toUs(fromMs) && e.text.find(part) != std::string::npos) return true;
  return false;
}
double Simulation::maxLineOffset(double fromMs, double toMs) const {
  double m = 0;
  for (const auto& s : samples_)
    if (s.t >= toUs(fromMs) && s.t <= toUs(toMs)) m = std::max(m, s.lineOffset);
  return m;
}

MapScore Simulation::scoreMap(double cellSize) const {
  MapScore m;
  m.cellSize = cellSize;
  const int nx = std::max(1, static_cast<int>(std::ceil(world_.width / cellSize)));
  const int ny = std::max(1, static_cast<int>(std::ceil(world_.height / cellSize)));
  m.cellsTotal = nx * ny;
  std::vector<bool> visited(static_cast<size_t>(m.cellsTotal), false);
  std::vector<std::pair<long, long>> unique;
  double pos = 0, temp = 0, map = 0;
  for (const auto& p : mapPoints_) {
    ++m.points;
    const double e = std::hypot(p.x - p.truePose.x, p.y - p.truePose.y);
    pos += e;
    m.maxPosError = std::max(m.maxPosError, e);
    temp += std::abs(p.temp - world_.temperature(Vec2{p.truePose.x, p.truePose.y}));
    map += std::abs(p.temp - world_.temperature(Vec2{p.x, p.y}));
    const std::pair<long, long> key(std::lround(p.x * 100), std::lround(p.y * 100));
    if (std::find(unique.begin(), unique.end(), key) == unique.end()) unique.push_back(key);
    const int cx = static_cast<int>(std::floor(p.x / cellSize)), cy = static_cast<int>(std::floor(p.y / cellSize));
    if (cx >= 0 && cy >= 0 && cx < nx && cy < ny) visited[static_cast<size_t>(cy * nx + cx)] = true;
  }
  for (bool v : visited) m.cellsVisited += v ? 1 : 0;
  m.coverage = static_cast<double>(m.cellsVisited) / m.cellsTotal;
  m.uniquePoints = static_cast<int>(unique.size());
  m.pointsPerM2 = m.uniquePoints / (world_.width * world_.height);
  if (m.points) {
    m.meanPosError = pos / m.points;
    m.meanTempError = temp / m.points;
    m.meanMapError = map / m.points;
  }
  return m;
}

void Simulation::check(bool ok, const std::string& text, const char* file, int line) {
  std::string where = file;
  const auto slash = where.find_last_of("/\\");
  if (slash != std::string::npos) where = where.substr(slash + 1);
  where += ":" + std::to_string(line);
  checks_.push_back(CheckResult{ok, text, where, nowUs_});
  if (verbose) std::cout << (ok ? "  PASS  " : "  FAIL  ") << text << "  (" << where << ")\n";
}
bool Simulation::passed() const {
  if (aborted_) return false;
  for (const auto& c : checks_)
    if (!c.ok) return false;
  return true;
}

// ------------------------------------------------------------- scenarios ---
static std::vector<ScenarioEntry>& scenarioList() {
  static std::vector<ScenarioEntry> list;
  return list;
}
void registerScenario(const char* name, const char* description, void (*run)(Simulation&)) {
  scenarioList().push_back(ScenarioEntry{name, description, run});
}
const std::vector<ScenarioEntry>& scenarios() {
  auto& l = scenarioList();
  std::sort(l.begin(), l.end(), [](const ScenarioEntry& a, const ScenarioEntry& b) { return a.name < b.name; });
  return l;
}

}  // namespace fwsim
