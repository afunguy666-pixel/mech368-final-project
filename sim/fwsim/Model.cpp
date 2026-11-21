#include "Model.h"
#include <Arduino_APDS9960.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>

namespace fwsim {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

double dist(Vec2 a, Vec2 b) { return std::hypot(a.x - b.x, a.y - b.y); }
double segmentDistance(Vec2 p, Vec2 a, Vec2 b) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  double t = len2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  return dist(p, Vec2{a.x + t * dx, a.y + t * dy});
}
// 射线 o + t*d（|d| = 1）与线段 ab 求交；返回 t 或 +inf。
double raySegment(Vec2 o, Vec2 d, Vec2 a, Vec2 b) {
  const double ex = b.x - a.x, ey = b.y - a.y;
  const double den = d.x * ey - d.y * ex;
  if (std::abs(den) < 1e-12) return kInf;
  const double ax = a.x - o.x, ay = a.y - o.y;
  const double t = (ax * ey - ay * ex) / den;
  const double s = (ax * d.y - ay * d.x) / den;
  return (t >= 0 && s >= 0 && s <= 1) ? t : kInf;
}
double rayCircle(Vec2 o, Vec2 d, Vec2 c, double r) {
  const double fx = o.x - c.x, fy = o.y - c.y;
  const double b = fx * d.x + fy * d.y;
  const double cc = fx * fx + fy * fy - r * r;
  const double disc = b * b - cc;
  if (disc < 0) return kInf;
  const double s = std::sqrt(disc);
  const double t1 = -b - s, t2 = -b + s;
  if (t1 >= 0) return t1;
  if (t2 >= 0) return 0.0;  // 起点在圆内
  return kInf;
}

// ------------------------------------------------------- 简易文件解析器 ---
struct Statement {
  std::string file;
  int line = 0;
  std::vector<std::string> words;          // 按位置的参数
  std::map<std::string, std::string> kv;   // 键=值
  std::string where() const { return file + ":" + std::to_string(line) + ": "; }
  [[noreturn]] void fail(const std::string& m) const { throw std::runtime_error(where() + m); }
  double num(size_t i) const {
    if (i >= words.size()) fail("'" + words[0] + "' 后面应该是一个数字");
    return parseNumber(words[i]);
  }
  double num(const std::string& key, double fallback) const {
    auto it = kv.find(key);
    return it == kv.end() ? fallback : parseNumber(it->second);
  }
  int pin(const std::string& key, bool required) const {
    auto it = kv.find(key);
    if (it == kv.end()) {
      if (required) fail("缺少 " + key + "=<引脚>");
      return -1;
    }
    const int p = parsePin(it->second);
    if (p < 0) fail("未知的引脚 '" + it->second + "'");
    return p;
  }
  bool flag(const std::string& name) const {
    return std::find(words.begin(), words.end(), name) != words.end();
  }
  double parseNumber(const std::string& s) const {
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0' || !std::isfinite(v)) fail("不是数字：'" + s + "'");
    return v;
  }
};

std::vector<Statement> readStatements(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("无法打开 " + path);
  std::vector<Statement> out;
  std::string text;
  int number = 0;
  while (std::getline(in, text)) {
    ++number;
    const auto hash = text.find('#');
    if (hash != std::string::npos) text.erase(hash);
    std::istringstream words(text);
    Statement st;
    st.file = path;
    st.line = number;
    std::string w;
    while (words >> w) {
      const auto eq = w.find('=');
      if (eq != std::string::npos && eq > 0) st.kv[w.substr(0, eq)] = w.substr(eq + 1);
      else st.words.push_back(w);
    }
    if (!st.words.empty()) out.push_back(st);
  }
  return out;
}
}  // namespace

int parsePin(const std::string& text) {
  static const std::map<std::string, int> named = {
      {"LED_BUILTIN", 13}, {"LEDR", 22}, {"LEDG", 23}, {"LEDB", 24}, {"LED_PWR", 25}};
  auto it = named.find(text);
  if (it != named.end()) return it->second;
  if (text.size() >= 2 && (text[0] == 'D' || text[0] == 'A')) {
    char* end = nullptr;
    const long n = std::strtol(text.c_str() + 1, &end, 10);
    if (*end == '\0' && n >= 0) {
      if (text[0] == 'D' && n <= 13) return static_cast<int>(n);
      if (text[0] == 'A' && n <= 7) return static_cast<int>(14 + n);
    }
    return -1;
  }
  char* end = nullptr;
  const long n = std::strtol(text.c_str(), &end, 10);
  return (end != text.c_str() && *end == '\0' && n >= 0 && n < kMaxPins) ? static_cast<int>(n) : -1;
}

// ----------------------------------------------------------------- 场地 ---
World World::load(const std::string& path) {
  World w;
  w.source = path;
  for (const auto& st : readStatements(path)) {
    const std::string& cmd = st.words[0];
    if (cmd == "size") {
      w.width = st.num(1);
      w.height = st.num(2);
    } else if (cmd == "start") {
      w.start = Pose2{st.num(1), st.num(2), st.num(3) * kPi / 180.0};
    } else if (cmd == "line") {
      Line l;
      l.width = st.num(1);
      if (st.words.size() < 6 || st.words.size() % 2 != 0)
        st.fail("line 宽度 x1 y1 x2 y2 [...] 至少需要两个点");
      for (size_t i = 2; i + 1 < st.words.size(); i += 2) l.pts.push_back(Vec2{st.num(i), st.num(i + 1)});
      w.lines.push_back(l);
    } else if (cmd == "stadium") {
      // 闭合椭圆环：两条直线段由半圆连接，中心在 (cx, cy)。
      const double cx = st.num(1), cy = st.num(2), straight = st.num(3), r = st.num(4);
      Line l;
      l.width = st.num(5);
      const int arcSegments = 48;
      const double h = straight / 2.0;
      l.pts.push_back(Vec2{cx - h, cy - r});
      l.pts.push_back(Vec2{cx + h, cy - r});
      for (int i = 1; i <= arcSegments; ++i) {
        const double a = -kPi / 2 + kPi * i / arcSegments;
        l.pts.push_back(Vec2{cx + h + r * std::cos(a), cy + r * std::sin(a)});
      }
      l.pts.push_back(Vec2{cx - h, cy + r});
      for (int i = 1; i <= arcSegments; ++i) {
        const double a = kPi / 2 + kPi * i / arcSegments;
        l.pts.push_back(Vec2{cx - h + r * std::cos(a), cy + r * std::sin(a)});
      }
      w.lines.push_back(l);
    } else if (cmd == "box") {
      Box b{Vec2{std::min(st.num(1), st.num(3)), std::min(st.num(2), st.num(4))},
            Vec2{std::max(st.num(1), st.num(3)), std::max(st.num(2), st.num(4))},
            st.words.size() > 5 ? st.words[5] : "方块"};
      w.boxes.push_back(b);
    } else if (cmd == "circle") {
      w.circles.push_back(Circle{Vec2{st.num(1), st.num(2)}, st.num(3),
                                 st.words.size() > 4 ? st.words[4] : "圆形障碍"});
    } else if (cmd == "wall") {
      w.walls.push_back(Wall{Vec2{st.num(1), st.num(2)}, Vec2{st.num(3), st.num(4)},
                             st.words.size() > 5 ? st.words[5] : "墙"});
    } else if (cmd == "border") {
      const Vec2 c[4] = {{0, 0}, {w.width, 0}, {w.width, w.height}, {0, w.height}};
      for (int i = 0; i < 4; ++i) w.walls.push_back(Wall{c[i], c[(i + 1) % 4], "边界"});
    } else if (cmd == "ambient") {
      w.ambient = st.num(1);
    } else if (cmd == "heat") {
      Heat h{Vec2{st.num(1), st.num(2)}, st.num(3), st.num(4), st.words.size() > 5 ? st.words[5] : "温度源"};
      if (h.radius <= 0) st.fail("heat 的半径必须为正数");
      w.heat.push_back(h);
    } else if (cmd == "rounded_border") {
      // 场地四周的墙，四角是半径为 R 的圆角。
      const double r = std::max(0.0, std::min(st.num(1), std::min(w.width, w.height) / 2));
      const Vec2 centres[4] = {{w.width - r, r}, {w.width - r, w.height - r}, {r, w.height - r}, {r, r}};
      const int arcSegments = 12;
      std::vector<Vec2> loop;
      for (int corner = 0; corner < 4; ++corner)
        for (int i = 0; i <= arcSegments; ++i) {
          const double a = -kPi / 2 + kPi / 2 * corner + kPi / 2 * i / arcSegments;
          loop.push_back(Vec2{centres[corner].x + r * std::cos(a), centres[corner].y + r * std::sin(a)});
        }
      for (size_t i = 0; i < loop.size(); ++i)
        w.walls.push_back(Wall{loop[i], loop[(i + 1) % loop.size()], "边界"});
    } else {
      st.fail("未知的场地命令 '" + cmd + "'");
    }
  }
  return w;
}

double World::lineCentreDistance(Vec2 p) const {
  double best = kInf;
  for (const auto& l : lines)
    for (size_t i = 0; i + 1 < l.pts.size(); ++i)
      best = std::min(best, segmentDistance(p, l.pts[i], l.pts[i + 1]));
  return best;
}

double World::lineCoverage(Vec2 p, double spot) const {
  double best = 0.0;
  for (const auto& l : lines) {
    double d = kInf;
    for (size_t i = 0; i + 1 < l.pts.size(); ++i) d = std::min(d, segmentDistance(p, l.pts[i], l.pts[i + 1]));
    // 光斑跨在胶带边缘时，读数线性过渡。
    const double edge = l.width / 2.0;
    const double c = spot > 0 ? (edge + spot - d) / (2.0 * spot) : (d <= edge ? 1.0 : 0.0);
    best = std::max(best, std::max(0.0, std::min(1.0, c)));
  }
  return best;
}

double World::raycast(Vec2 o, double yaw, double maxRange) const {
  const Vec2 d{std::cos(yaw), std::sin(yaw)};
  double best = kInf;
  for (const auto& w : walls) best = std::min(best, raySegment(o, d, w.a, w.b));
  for (const auto& b : boxes) {
    const Vec2 c[4] = {b.lo, {b.hi.x, b.lo.y}, b.hi, {b.lo.x, b.hi.y}};
    for (int i = 0; i < 4; ++i) best = std::min(best, raySegment(o, d, c[i], c[(i + 1) % 4]));
  }
  for (const auto& c : circles) best = std::min(best, rayCircle(o, d, c.c, c.r));
  return best <= maxRange ? best : kInf;
}

bool World::collides(Vec2 p, double r, std::string* what) const {
  for (const auto& b : boxes) {
    const double cx = std::max(b.lo.x, std::min(p.x, b.hi.x));
    const double cy = std::max(b.lo.y, std::min(p.y, b.hi.y));
    if (std::hypot(p.x - cx, p.y - cy) < r) { if (what) *what = b.name; return true; }
  }
  for (const auto& c : circles)
    if (dist(p, c.c) < r + c.r) { if (what) *what = c.name; return true; }
  for (const auto& w : walls)
    if (segmentDistance(p, w.a, w.b) < r) { if (what) *what = w.name; return true; }
  return false;
}

double World::temperature(Vec2 p) const {
  double t = ambient;
  for (const auto& h : heat) {
    const double d = dist(p, h.c) / h.radius;
    t += h.delta * std::exp(-d * d);
  }
  return t;
}

double World::lineLength() const {
  double total = 0;
  for (const auto& l : lines)
    for (size_t i = 0; i + 1 < l.pts.size(); ++i) total += dist(l.pts[i], l.pts[i + 1]);
  return total;
}

// --------------------------------------------------------- 小车配置 ---
VehicleConfig VehicleConfig::load(const std::string& path) {
  VehicleConfig v;
  v.source = path;
  auto side = [](const Statement& st) -> int {
    if (st.words.size() < 2 || (st.words[1] != "left" && st.words[1] != "right"))
      st.fail("'" + st.words[0] + "' 后面应该是 'left' 或 'right'");
    return st.words[1] == "left" ? 0 : 1;
  };
  for (const auto& st : readStatements(path)) {
    const std::string& cmd = st.words[0];
    if (cmd == "track_width") v.trackWidth = st.num(1);
    else if (cmd == "wheel_radius") v.wheelRadius = st.num(1);
    else if (cmd == "body_radius") v.bodyRadius = st.num(1);
    else if (cmd == "motor_max_speed") v.maxWheelSpeed = st.num(1);
    else if (cmd == "motor_time_constant") v.timeConstant = st.num(1);
    else if (cmd == "motor_deadband") v.deadband = st.num(1);
    else if (cmd == "nominal_voltage") v.nominalVolts = st.num(1);
    else if (cmd == "wheel_scale") (side(st) == 0 ? v.leftScale : v.rightScale) = st.num(2);
    else if (cmd == "gyro") {
      v.gyroBiasDps = st.num("bias", 0.0);
      v.gyroNoiseDps = st.num("noise", 0.0);
    } else if (cmd == "motor") {
      MotorConfig& m = side(st) == 0 ? v.left : v.right;
      if (st.words.size() < 3) st.fail("格式：motor 左右 驱动类型 引脚...（驱动类型：pwm_dir、in_in、en_in_in）");
      const std::string& driver = st.words[2];
      if (driver == "pwm_dir") {
        m.driver = Driver::PwmDir;
        m.pwm = st.pin("pwm", true);
        m.dir = st.pin("dir", true);
      } else if (driver == "in_in") {
        m.driver = Driver::InIn;
        m.in1 = st.pin("in1", true);
        m.in2 = st.pin("in2", true);
      } else if (driver == "en_in_in") {
        m.driver = Driver::EnInIn;
        m.en = st.pin("en", true);
        m.in1 = st.pin("in1", true);
        m.in2 = st.pin("in2", true);
      } else {
        st.fail("未知的电机驱动类型 '" + driver + "'（可选：pwm_dir、in_in、en_in_in）");
      }
      m.invert = st.flag("invert");
    } else if (cmd == "encoder") {
      EncoderConfig& e = side(st) == 0 ? v.encLeft : v.encRight;
      e.a = st.pin("a", true);
      e.b = st.pin("b", false);
      e.ppr = st.num("ppr", 0);
      if (e.ppr <= 0) st.fail("encoder 需要 ppr=<车轮每转一圈 A 相的脉冲数>");
      e.invert = st.flag("invert");
    } else if (cmd == "line_sensor") {
      LineSensorConfig s;
      s.name = st.words.size() > 1 ? st.words[1] : "line" + std::to_string(v.lineSensors.size());
      s.pin = st.pin("pin", true);
      s.analog = !st.flag("digital");
      s.x = st.num("x", 0);
      s.y = st.num("y", 0);
      s.onVolts = st.num("on", s.onVolts);
      s.offVolts = st.num("off", s.offVolts);
      s.onLevel = static_cast<int>(st.num("on_level", 1));
      s.spot = st.num("spot", s.spot);
      v.lineSensors.push_back(s);
    } else if (cmd == "ultrasonic" || cmd == "ir_distance") {
      DistanceConfig d;
      d.kind = cmd == "ultrasonic" ? DistanceConfig::Ultrasonic : DistanceConfig::AnalogIR;
      d.name = st.words.size() > 1 ? st.words[1] : cmd;
      if (d.kind == DistanceConfig::Ultrasonic) {
        d.trig = st.pin("trig", true);
        d.echo = st.pin("echo", true);
      } else {
        d.pin = st.pin("pin", true);
        d.minRange = 0.10;
        d.maxRange = 0.80;
      }
      d.x = st.num("x", 0);
      d.y = st.num("y", 0);
      d.yaw = st.num("yaw_deg", 0) * kPi / 180.0;
      d.minRange = st.num("min", d.minRange);
      d.maxRange = st.num("max", d.maxRange);
      d.noise = st.num("noise", d.noise);
      v.distance.push_back(d);
    } else if (cmd == "temperature") {
      v.tempTau = st.num("tau", v.tempTau);
      v.tempNoise = st.num("noise", v.tempNoise);
      v.tempOffset = st.num("offset", v.tempOffset);
      if (v.tempTau <= 0) st.fail("temperature 的 tau 必须为正数（秒）");
    } else if (cmd == "battery") {
      v.battery.pin = st.pin("pin", false);
      v.battery.divider = st.num("divider", v.battery.divider);
      v.battery.volts = st.num("volts", v.battery.volts);
      v.battery.drainPerMinute = st.num("drain_per_min", 0.0);
    } else {
      st.fail("未知的小车命令 '" + cmd + "'");
    }
  }
  if (v.trackWidth <= 0 || v.wheelRadius <= 0 || v.maxWheelSpeed <= 0 || v.timeConstant <= 0)
    throw std::runtime_error(path + "：几何参数和电机参数必须为正数");
  return v;
}

// ------------------------------------------------------------ ScriptedIO ---
bool ScriptedIO::gestureDue(Board& b) {
  for (const auto& g : gestures)
    if (g.first <= b.now()) return true;
  return false;
}
int ScriptedIO::takeGesture(Board& b) {
  for (auto it = gestures.begin(); it != gestures.end(); ++it)
    if (it->first <= b.now()) {
      const int g = it->second;
      gestures.erase(it);
      return g;
    }
  return GESTURE_NONE;
}
Vec2 ScriptedIO::tiltAt(uint64_t t) const {
  if (tilt.empty()) return Vec2{0, 0};
  if (t <= tilt.front().first) return tilt.front().second;
  for (size_t i = 1; i < tilt.size(); ++i)
    if (t <= tilt[i].first) {
      const auto& a = tilt[i - 1];
      const auto& b = tilt[i];
      const double f = b.first == a.first ? 1.0
                       : static_cast<double>(t - a.first) / static_cast<double>(b.first - a.first);
      return Vec2{a.second.x + f * (b.second.x - a.second.x),
                  a.second.y + f * (b.second.y - a.second.y)};
    }
  return tilt.back().second;
}
ImuSample ScriptedIO::imu(Board& b) {
  // roll > 0：右边缘向下；pitch > 0：前边缘向下。开发板水平时 = (0,0,1) g。
  const uint64_t t = b.now();
  const Vec2 a = tiltAt(t);
  const Vec2 before = tiltAt(t >= 10000 ? t - 10000 : 0);
  const double roll = a.x * kPi / 180.0, pitch = a.y * kPi / 180.0;
  ImuSample s;
  s.ax = static_cast<float>(-std::sin(pitch));
  s.ay = static_cast<float>(std::sin(roll) * std::cos(pitch));
  s.az = static_cast<float>(std::cos(roll) * std::cos(pitch));
  s.gx = static_cast<float>((a.x - before.x) / 0.01);
  s.gy = static_cast<float>((a.y - before.y) / 0.01);
  s.gz = 0;
  return s;
}

// --------------------------------------------------------------- 小车 ---
Vehicle::Vehicle(const VehicleConfig& config, const World& w)
    : cfg(config), world(w), pose(w.start), batteryVolts(config.battery.volts),
      sensedTemp(w.temperature(Vec2{w.start.x, w.start.y})) {
  trigRise.assign(cfg.distance.size(), kNever);
  trigFall.assign(cfg.distance.size(), kNever);
}

double Vehicle::motorCommand(const MotorConfig& m) const {
  if (!board || !board->started || board->poweredOff) return 0.0;
  const Board& b = *board;
  auto level = [&](int pin) { return b.outputFraction(pin) > 0.5 ? 1.0 : 0.0; };
  double u = 0.0;
  switch (m.driver) {
    case Driver::None: return 0.0;
    case Driver::PwmDir: u = b.outputFraction(m.pwm) * (level(m.dir) > 0 ? 1.0 : -1.0); break;
    case Driver::InIn: u = b.outputFraction(m.in1) - b.outputFraction(m.in2); break;
    case Driver::EnInIn: u = b.outputFraction(m.en) * (level(m.in1) - level(m.in2)); break;
  }
  return m.invert ? -u : u;
}

void Vehicle::encoderEdges(const EncoderConfig& e, long long& index, double theta, uint64_t t0) {
  if (e.a < 0 || e.ppr <= 0) return;
  const double sign = e.invert ? -1.0 : 1.0;
  const long long target = static_cast<long long>(std::floor(sign * theta * 4.0 * e.ppr / (2.0 * kPi)));
  const long long steps = std::llabs(target - index);
  if (steps == 0) return;
  const long long dir = target > index ? 1 : -1;
  for (long long k = 1; k <= steps; ++k) {
    index += dir;
    const int m = static_cast<int>(((index % 4) + 4) % 4);
    const int a = (m == 1 || m == 2) ? 1 : 0;
    const int bl = (m == 2 || m == 3) ? 1 : 0;
    const uint64_t t = t0 + static_cast<uint64_t>(stepUs * k / (steps + 1));
    if (board && board->started && !board->poweredOff) {
      board->setInputLevel(e.a, a, t);
      if (e.b >= 0) board->setInputLevel(e.b, bl, t);
    }
  }
}

void Vehicle::step() {
  const double dt = stepUs / 1e6;
  dutyL = motorCommand(cfg.left);
  dutyR = motorCommand(cfg.right);
  const double supply = cfg.nominalVolts > 0 ? batteryVolts / cfg.nominalVolts : 1.0;
  const double alpha = std::min(1.0, dt / cfg.timeConstant);
  auto target = [&](double u) {
    u = std::max(-1.0, std::min(1.0, u));
    return std::abs(u) < cfg.deadband ? 0.0 : u * cfg.maxWheelSpeed * supply;
  };
  omegaL += (target(dutyL) - omegaL) * alpha;
  omegaR += (target(dutyR) - omegaR) * alpha;
  const uint64_t t0 = timeUs;
  thetaL += omegaL * dt;
  thetaR += omegaR * dt;
  encoderEdges(cfg.encLeft, encIndexL, thetaL, t0);
  encoderEdges(cfg.encRight, encIndexR, thetaR, t0);

  const double dl = omegaL * dt * cfg.wheelRadius * cfg.leftScale;
  const double dr = omegaR * dt * cfg.wheelRadius * cfg.rightScale;
  const double ds = (dl + dr) / 2.0, dyaw = (dr - dl) / cfg.trackWidth;
  Pose2 next = pose;
  const double half = dyaw / 2.0;
  const double sinc = std::abs(half) < 1e-9 ? 1.0 : std::sin(half) / half;
  next.x += ds * sinc * std::cos(pose.yaw + half);
  next.y += ds * sinc * std::sin(pose.yaw + half);
  next.yaw += dyaw;
  std::string what;
  const bool hit = world.collides(Vec2{next.x, next.y}, cfg.bodyRadius, &what);
  const double previousSpeed = speed;
  if (hit) {
    // 车身被挡住：车轮打滑，底盘位置不变，但可以转动。
    next.x = pose.x;
    next.y = pose.y;
    if (!colliding) {
      ++collisions;
      if (onEvent) onEvent(timeUs, "碰撞：" + what);
    }
    speed = 0;
  } else {
    speed = ds / dt;
    travelled += std::abs(ds);
  }
  colliding = hit;
  yawRate = dyaw / dt;
  accel = (speed - previousSpeed) / dt;
  pose = next;
  batteryVolts = std::max(0.0, batteryVolts - cfg.battery.drainPerMinute * dt / 60.0);
  sensedTemp += (world.temperature(Vec2{pose.x, pose.y}) - sensedTemp) * std::min(1.0, dt / cfg.tempTau);
  timeUs += stepUs;
  if (onSample && timeUs >= nextSampleUs) {
    onSample(timeUs);
    nextSampleUs = timeUs + sampleEveryUs;
  }
}

void Vehicle::advanceTo(uint64_t t) {
  while (timeUs + stepUs <= t) step();
}

Vec2 Vehicle::sensorPosition(double x, double y) const {
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  return Vec2{pose.x + c * x - s * y, pose.y + s * x + c * y};
}

double Vehicle::lineSensorCoverage(size_t i) const {
  const auto& s = cfg.lineSensors.at(i);
  return world.lineCoverage(sensorPosition(s.x, s.y), s.spot);
}

double Vehicle::distanceReading(size_t i) const {
  const auto& d = cfg.distance.at(i);
  const Vec2 o = sensorPosition(d.x, d.y);
  const double r = world.raycast(o, pose.yaw + d.yaw, d.maxRange);
  return r < d.minRange ? d.minRange : r;
}

double Vehicle::lineOffset() const {
  if (cfg.lineSensors.empty()) return world.lineCentreDistance(Vec2{pose.x, pose.y});
  double x = 0, y = 0;
  for (const auto& s : cfg.lineSensors) { x += s.x; y += s.y; }
  const double n = static_cast<double>(cfg.lineSensors.size());
  return world.lineCentreDistance(sensorPosition(x / n, y / n));
}

bool Vehicle::analogInput(Board&, int pin, double& volts) {
  std::uniform_real_distribution<double> noise(-0.02, 0.02);
  for (size_t i = 0; i < cfg.lineSensors.size(); ++i) {
    const auto& s = cfg.lineSensors[i];
    if (s.pin != pin || !s.analog) continue;
    volts = s.offVolts + (s.onVolts - s.offVolts) * lineSensorCoverage(i) + noise(rng);
    return true;
  }
  for (size_t i = 0; i < cfg.distance.size(); ++i) {
    const auto& d = cfg.distance[i];
    if (d.kind != DistanceConfig::AnalogIR || d.pin != pin) continue;
    double r = distanceReading(i);
    if (!std::isfinite(r)) r = d.maxRange;
    r = std::max(d.minRange, std::min(d.maxRange, r + d.noise * gauss(rng)));
    volts = std::pow(27.728 / (r * 100.0), 1.0 / 1.2045);  // 类似 Sharp GP2Y0A21 的曲线
    return true;
  }
  if (cfg.battery.pin == pin) {
    volts = batteryVolts / cfg.battery.divider;
    return true;
  }
  return false;
}

bool Vehicle::digitalInput(Board&, int pin, int& level) {
  for (size_t i = 0; i < cfg.lineSensors.size(); ++i) {
    const auto& s = cfg.lineSensors[i];
    if (s.pin != pin || s.analog) continue;
    const bool on = lineSensorCoverage(i) > 0.5;
    level = on ? s.onLevel : 1 - s.onLevel;
    return true;
  }
  return false;
}

void Vehicle::outputChanged(Board& b, int pin) {
  for (size_t i = 0; i < cfg.distance.size(); ++i) {
    if (cfg.distance[i].kind != DistanceConfig::Ultrasonic || cfg.distance[i].trig != pin) continue;
    const int level = b.outputLevel(pin);
    if (level && trigRise[i] == kNever) trigRise[i] = b.now();
    if (!level && trigRise[i] != kNever) {
      if (b.now() - trigRise[i] >= 8) trigFall[i] = b.now();  // >= 约 10 us 的触发脉冲
      trigRise[i] = kNever;
    }
  }
}

bool Vehicle::pulseIn(Board& b, int pin, int state, uint64_t timeoutUs, uint64_t& durationUs) {
  for (size_t i = 0; i < cfg.distance.size(); ++i) {
    const auto& d = cfg.distance[i];
    if (d.kind != DistanceConfig::Ultrasonic || d.echo != pin) continue;
    const bool triggered = state == HIGH && trigFall[i] != kNever && b.now() - trigFall[i] < 5000;
    trigFall[i] = kNever;
    double r = triggered ? distanceReading(i) : std::numeric_limits<double>::infinity();
    const uint64_t burstUs = 450;  // 回波引脚拉高前换能器发出脉冲串的时间
    if (!std::isfinite(r)) {
      b.spend(timeoutUs);
      durationUs = 0;
      return true;
    }
    r = std::max(d.minRange, r + d.noise * gauss(rng));
    const uint64_t echoUs = static_cast<uint64_t>(2.0 * r / 343.0 * 1e6);
    if (burstUs + echoUs > timeoutUs) {
      b.spend(timeoutUs);
      durationUs = 0;
      return true;
    }
    b.spend(burstUs + echoUs);
    durationUs = echoUs;
    return true;
  }
  return false;
}

bool Vehicle::temperature(Board&, double& celsius) {
  celsius = sensedTemp + cfg.tempOffset + cfg.tempNoise * gauss(rng);
  return true;
}

ImuSample Vehicle::imu(Board&) {
  ImuSample s;
  s.ax = static_cast<float>(accel / 9.80665);
  s.ay = static_cast<float>(speed * yawRate / 9.80665);
  s.az = 1.0f;
  s.gx = static_cast<float>(cfg.gyroNoiseDps * gauss(rng));
  s.gy = static_cast<float>(cfg.gyroNoiseDps * gauss(rng));
  s.gz = static_cast<float>(yawRate * 180.0 / kPi + cfg.gyroBiasDps + cfg.gyroNoiseDps * gauss(rng));
  // 水平分量指向世界坐标 +y，20 uT；垂直分量向下，40 uT。
  s.mx = static_cast<float>(20.0 * std::sin(pose.yaw));
  s.my = static_cast<float>(20.0 * std::cos(pose.yaw));
  s.mz = -40.0f;
  return s;
}

}  // namespace fwsim
