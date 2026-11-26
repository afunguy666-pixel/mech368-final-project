// Writes <name>.csv, <name>.log and a self-contained <name>.html replay.
#include "Simulation.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace fwsim {

namespace {
const char* kViewerTemplate =
#include "viewer_html.inc"
    ;

std::string json(const std::string& s) {
  std::string out = "\"";
  for (unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': break;
      case '\t': out += "\\t"; break;
      case '<': out += "\\u003c"; break;  // keep </script> out of the page
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof buf, "\\u%04x", c);
          out += buf;
        } else {
          out += static_cast<char>(c);
        }
    }
  }
  return out + "\"";
}
std::string num(double v, int digits = 4) {
  if (!std::isfinite(v)) return "null";
  char buf[48];
  std::snprintf(buf, sizeof buf, "%.*f", digits, v);
  std::string s = buf;
  if (s.find('.') != std::string::npos) {
    while (!s.empty() && s.back() == '0') s.pop_back();
    if (!s.empty() && s.back() == '.') s.pop_back();
  }
  return s == "-0" ? "0" : s;
}
template <class F>
std::string column(const std::vector<Sample>& samples, F f, int digits = 4) {
  std::string out = "[";
  for (size_t i = 0; i < samples.size(); ++i) {
    if (i) out += ",";
    out += num(f(samples[i]), digits);
  }
  return out + "]";
}
std::string pt(Vec2 p) { return "[" + num(p.x) + "," + num(p.y) + "]"; }
}  // namespace

void Simulation::writeReport(const std::string& dir, const std::string& name,
                             const std::string& description) const {
  const std::string base = dir + "/" + name;
  const auto& vc = vehicle_ ? vehicle_->cfg : vehicleConfig_;

  // ---------------------------------------------------------------- CSV ---
  {
    std::ofstream f(base + ".csv");
    f << "t_ms,x_m,y_m,yaw_rad,left_rad_s,right_rad_s,left_duty,right_duty,battery_v,"
         "line_offset_m,robot_led,remote_led,robot_link,remote_link,status_flags,colliding,"
         "temp_c,true_temp_c";
    for (const auto& s : vc.lineSensors) f << ",line_" << s.name;
    for (const auto& d : vc.distance) f << ",dist_" << d.name << "_m";
    f << "\n";
    for (const auto& s : samples_) {
      f << num(s.t / 1000.0, 1) << ',' << num(s.pose.x, 5) << ',' << num(s.pose.y, 5) << ','
        << num(s.pose.yaw, 5) << ',' << num(s.omegaL, 3) << ',' << num(s.omegaR, 3) << ','
        << num(s.dutyL, 3) << ',' << num(s.dutyR, 3) << ',' << num(s.battery, 3) << ','
        << num(s.lineOffset, 4) << ',' << s.robotLed << ',' << s.remoteLed << ','
        << s.robotLink << ',' << s.remoteLink << ',' << s.statusFlags << ',' << s.colliding << ','
        << num(s.temp, 3) << ',' << num(s.tempTrue, 3);
      for (double v : s.line) f << ',' << num(v, 3);
      for (double v : s.distance) f << ',' << (std::isfinite(v) ? num(v, 4) : "");
      f << "\n";
    }
  }

  // ---------------------------------------------------------- map points ---
  if (!mapPoints_.empty()) {
    std::ofstream f(base + ".map.csv");
    f << "t_ms,board,logged_x_m,logged_y_m,logged_temp_c,true_x_m,true_y_m,true_temp_c\n";
    for (const auto& p : mapPoints_)
      f << num(p.t / 1000.0, 1) << ',' << boards_[p.board]->name << ',' << num(p.x) << ',' << num(p.y) << ','
        << num(p.temp, 2) << ',' << num(p.truePose.x) << ',' << num(p.truePose.y) << ','
        << num(world_.temperature(Vec2{p.truePose.x, p.truePose.y}), 2) << "\n";
  }

  // ------------------------------------------------------ radio frames ---
  {
    static const char* fates[] = {"delivered", "lost", "blocked", "corrupted", "link_down"};
    static const char* types[] = {"?", "motion", "estop", "status"};
    std::ofstream f(base + ".frames.csv");
    f << "sent_ms,deliver_ms,direction,fate,decoded,type,sequence,session,challenge,"
         "linear_or_flags,angular\n";
    for (const auto& r : frames_) {
      const int type = static_cast<int>(r.frame.type);
      f << num(r.sentUs / 1000.0, 3) << ',' << (r.deliverUs ? num(r.deliverUs / 1000.0, 3) : "") << ','
        << (r.fromRemote ? "remote->robot" : "robot->remote") << ',' << fates[r.fate] << ','
        << r.decoded;
      if (r.decoded)
        f << ',' << types[type >= 1 && type <= 3 ? type : 0] << ',' << r.frame.sequence << ','
          << r.frame.session << ',' << r.frame.challenge << ',' << r.frame.linear << ','
          << r.frame.angular;
      else
        f << ",,,,,,";
      f << "\n";
    }
  }

  // ---------------------------------------------------------------- log ---
  struct Line { uint64_t t; std::string text; };
  std::vector<Line> lines;
  for (const auto& e : events_) lines.push_back(Line{e.t, e.source + ": " + e.text});
  for (const auto& b : boards_)
    for (const auto& l : b->serialOut) lines.push_back(Line{l.t, b->name + "> " + l.text});
  std::stable_sort(lines.begin(), lines.end(), [](const Line& a, const Line& b) { return a.t < b.t; });
  {
    std::ofstream f(base + ".log");
    f << "scenario: " << name << "\n" << description << "\n\n";
    for (const auto& l : lines) {
      char buf[32];
      std::snprintf(buf, sizeof buf, "[%9.3f s] ", l.t / 1e6);
      f << buf << l.text << "\n";
    }
    f << "\nchecks:\n";
    for (const auto& c : checks_) f << (c.ok ? "  PASS  " : "  FAIL  ") << c.text << "  (" << c.where << ")\n";
    f << (passed() ? "RESULT: PASS\n" : "RESULT: FAIL\n");
  }

  // --------------------------------------------------------------- HTML ---
  std::ostringstream d;
  d << "{\"name\":" << json(name) << ",\"description\":" << json(description)
    << ",\"passed\":" << (passed() ? "true" : "false")
    << ",\"sketches\":{\"robot\":" << json(boards_[Robot]->sketch ? boards_[Robot]->sketch->name : "")
    << ",\"remote\":" << json(boards_[Remote]->sketch ? boards_[Remote]->sketch->name : "") << "}";
  d << ",\"world\":{\"width\":" << num(world_.width) << ",\"height\":" << num(world_.height)
    << ",\"source\":" << json(world_.source) << ",\"lines\":[";
  for (size_t i = 0; i < world_.lines.size(); ++i) {
    d << (i ? "," : "") << "{\"w\":" << num(world_.lines[i].width) << ",\"pts\":[";
    for (size_t k = 0; k < world_.lines[i].pts.size(); ++k) d << (k ? "," : "") << pt(world_.lines[i].pts[k]);
    d << "]}";
  }
  d << "],\"boxes\":[";
  for (size_t i = 0; i < world_.boxes.size(); ++i) {
    const auto& b = world_.boxes[i];
    d << (i ? "," : "") << "[" << num(b.lo.x) << "," << num(b.lo.y) << "," << num(b.hi.x) << ","
      << num(b.hi.y) << "," << json(b.name) << "]";
  }
  d << "],\"circles\":[";
  for (size_t i = 0; i < world_.circles.size(); ++i) {
    const auto& c = world_.circles[i];
    d << (i ? "," : "") << "[" << num(c.c.x) << "," << num(c.c.y) << "," << num(c.r) << "," << json(c.name) << "]";
  }
  d << "],\"walls\":[";
  for (size_t i = 0; i < world_.walls.size(); ++i) {
    const auto& w = world_.walls[i];
    d << (i ? "," : "") << "[" << num(w.a.x) << "," << num(w.a.y) << "," << num(w.b.x) << "," << num(w.b.y) << "]";
  }
  d << "],\"ambient\":" << num(world_.ambient) << ",\"heat\":[";
  for (size_t i = 0; i < world_.heat.size(); ++i) {
    const auto& h = world_.heat[i];
    d << (i ? "," : "") << "[" << num(h.c.x) << "," << num(h.c.y) << "," << num(h.radius) << ","
      << num(h.delta) << "," << json(h.name) << "]";
  }
  d << "]},\"vehicle\":{\"track\":" << num(vc.trackWidth) << ",\"wheel\":" << num(vc.wheelRadius)
    << ",\"body\":" << num(vc.bodyRadius) << ",\"bodyX\":" << num(vc.bodyOffset) << ",\"source\":" << json(vc.source) << ",\"line\":[";
  for (size_t i = 0; i < vc.lineSensors.size(); ++i)
    d << (i ? "," : "") << "[" << num(vc.lineSensors[i].x) << "," << num(vc.lineSensors[i].y) << ","
      << json(vc.lineSensors[i].name) << "]";
  d << "],\"distance\":[";
  for (size_t i = 0; i < vc.distance.size(); ++i)
    d << (i ? "," : "") << "[" << num(vc.distance[i].x) << "," << num(vc.distance[i].y) << ","
      << num(vc.distance[i].yaw) << "," << num(vc.distance[i].maxRange) << "," << json(vc.distance[i].name) << "]";
  d << "]}";
  const auto& S = samples_;
  d << ",\"samples\":{\"t\":" << column(S, [](const Sample& s) { return s.t / 1000.0; }, 1)
    << ",\"x\":" << column(S, [](const Sample& s) { return s.pose.x; })
    << ",\"y\":" << column(S, [](const Sample& s) { return s.pose.y; })
    << ",\"yaw\":" << column(S, [](const Sample& s) { return s.pose.yaw; })
    << ",\"wl\":" << column(S, [](const Sample& s) { return s.omegaL; }, 2)
    << ",\"wr\":" << column(S, [](const Sample& s) { return s.omegaR; }, 2)
    << ",\"ul\":" << column(S, [](const Sample& s) { return s.dutyL; }, 3)
    << ",\"ur\":" << column(S, [](const Sample& s) { return s.dutyR; }, 3)
    << ",\"batt\":" << column(S, [](const Sample& s) { return s.battery; }, 2)
    << ",\"off\":" << column(S, [](const Sample& s) { return s.lineOffset; })
    << ",\"led\":" << column(S, [](const Sample& s) { return s.robotLed; }, 0)
    << ",\"rled\":" << column(S, [](const Sample& s) { return s.remoteLed; }, 0)
    << ",\"r\":" << column(S, [](const Sample& s) { return s.ledR; }, 0)
    << ",\"g\":" << column(S, [](const Sample& s) { return s.ledG; }, 0)
    << ",\"b\":" << column(S, [](const Sample& s) { return s.ledB; }, 0)
    << ",\"link\":" << column(S, [](const Sample& s) { return (s.robotLink ? 1 : 0) + (s.remoteLink ? 2 : 0); }, 0)
    << ",\"flags\":" << column(S, [](const Sample& s) { return s.statusFlags; }, 0)
    << ",\"hit\":" << column(S, [](const Sample& s) { return s.colliding ? 1 : 0; }, 0)
    << ",\"temp\":" << column(S, [](const Sample& s) { return s.temp; }, 2)
    << ",\"ttrue\":" << column(S, [](const Sample& s) { return s.tempTrue; }, 2)
    << ",\"line\":[";
  for (size_t i = 0; i < vc.lineSensors.size(); ++i)
    d << (i ? "," : "") << column(S, [i](const Sample& s) { return i < s.line.size() ? s.line[i] : 0.0; }, 2);
  d << "],\"dist\":[";
  for (size_t i = 0; i < vc.distance.size(); ++i)
    d << (i ? "," : "")
      << column(S, [i](const Sample& s) {
           return i < s.distance.size() ? s.distance[i] : std::numeric_limits<double>::infinity();
         }, 3);
  d << "]},\"log\":[";
  for (size_t i = 0; i < lines.size(); ++i)
    d << (i ? "," : "") << "[" << num(lines[i].t / 1000.0, 1) << "," << json(lines[i].text) << "]";
  d << "],\"map\":[";
  for (size_t i = 0; i < mapPoints_.size(); ++i) {
    const auto& p = mapPoints_[i];
    d << (i ? "," : "") << "[" << num(p.t / 1000.0, 1) << "," << num(p.x) << "," << num(p.y) << ","
      << num(p.temp, 2) << "," << num(p.truePose.x) << "," << num(p.truePose.y) << "]";
  }
  const MapScore ms = scoreMap();
  d << "],\"mapScore\":{\"points\":" << ms.points << ",\"unique\":" << ms.uniquePoints
    << ",\"perM2\":" << num(ms.pointsPerM2, 1) << ",\"coverage\":" << num(ms.coverage, 3)
    << ",\"cell\":" << num(ms.cellSize, 2) << ",\"posErr\":" << num(ms.meanPosError, 4)
    << ",\"maxPosErr\":" << num(ms.maxPosError, 4) << ",\"tempErr\":" << num(ms.meanTempError, 3)
    << ",\"mapErr\":" << num(ms.meanMapError, 3) << "}";
  d << ",\"checks\":[";
  for (size_t i = 0; i < checks_.size(); ++i)
    d << (i ? "," : "") << "[" << (checks_[i].ok ? "true" : "false") << "," << json(checks_[i].text) << ","
      << num(checks_[i].t / 1000.0, 1) << "]";
  d << "]}";

  std::string html = kViewerTemplate;
  const std::string marker = "/*FWSIM_DATA*/null";
  const auto at = html.find(marker);
  if (at != std::string::npos) html.replace(at, marker.size(), d.str());
  const std::string titleMarker = "FWSIM_TITLE";
  const auto title = html.find(titleMarker);
  if (title != std::string::npos) html.replace(title, titleMarker.size(), name);
  std::ofstream(base + ".html") << html;
}

}  // namespace fwsim
