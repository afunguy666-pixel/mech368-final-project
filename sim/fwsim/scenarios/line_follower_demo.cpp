// sim/examples/line_follower 的场景（演示模拟器中的电机、编码器、巡线传感器、
// 超声波、手势、IMU、电池和温度）。
#include "Simulation.h"
#include <Arduino_APDS9960.h>
#include <cmath>
#include <cstdio>

using namespace fwsim;

namespace {
void demo(Simulation& sim, const char* world) {
  sim.loadWorld(world);
  sim.loadVehicle("sim/vehicles/example.vehicle");
  sim.useSketch(Robot, "line_follower");
}
double speedAt(const Simulation& sim, double ms) {
  const auto& s = sim.samples();
  for (size_t i = 1; i < s.size(); ++i)
    if (s[i].t >= ms * 1000) {
      const double dt = (s[i].t - s[i - 1].t) / 1e6;
      return std::hypot(s[i].pose.x - s[i - 1].pose.x, s[i].pose.y - s[i - 1].pose.y) / dt;
    }
  return 0;
}
}  // namespace

FWSIM_SCENARIO(line_follow, "演示：手势启动，沿椭圆赛道跑一圈，手势停车") {
  demo(sim, "sim/worlds/oval.world");
  sim.gesture(1000, GESTURE_UP);
  sim.gesture(27000, GESTURE_DOWN);
  sim.runUntil(900);
  const Pose2 start = sim.vehicle().pose;
  FWSIM_CHECK(sim, std::hypot(start.x - sim.world().start.x, start.y - sim.world().start.y) < 1e-6,
              "做出 UP 手势之前机器人不动");
  sim.runUntil(27000);
  const double lap = sim.world().lineLength();
  char buf[128];
  std::snprintf(buf, sizeof buf, "行驶 %.2f m（一圈 %.2f m）；最大偏离 %.1f mm",
                sim.vehicle().travelled, lap, sim.maxLineOffset(3000, 27000) * 1000);
  sim.log("检查", buf);
  FWSIM_CHECK(sim, sim.vehicle().travelled >= lap, "26 s 内至少跑完一圈");
  FWSIM_CHECK(sim, sim.maxLineOffset(3000, 27000) < 0.015, "传感器排始终在线中心 15 mm 以内");
  FWSIM_CHECK(sim, sim.vehicle().collisions == 0, "没有碰撞");
  sim.runUntil(29000);
  FWSIM_CHECK(sim, speedAt(sim, 28900) < 0.01, "DOWN 手势后机器人停下");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "手势 DOWN"), "程序记录了 DOWN 手势");
}

FWSIM_SCENARIO(obstacle_stop, "演示：在赛道上的方块前停车，方块移走后继续") {
  demo(sim, "sim/worlds/oval_obstacle.world");
  sim.gesture(500, GESTURE_UP);
  sim.at(25000, [](Simulation& s) { s.world().boxes.clear(); }, "方块已从场地移走");
  sim.runUntil(24000);
  FWSIM_CHECK(sim, sim.vehicle().collisions == 0, "机器人始终没有碰到方块");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "前方障碍物"), "程序检测到了障碍物");
  FWSIM_CHECK(sim, speedAt(sim, 23900) < 0.01, "机器人停在方块前方");
  const double gap = sim.vehicle().distanceReading(0);
  char buf[96];
  std::snprintf(buf, sizeof buf, "停车时超声波测得距离：%.1f cm", gap * 100);
  sim.log("检查", buf);
  FWSIM_CHECK(sim, gap > 0.05 && gap < 0.25, "停在距方块 5 到 25 cm 处");
  sim.runUntil(28000);
  FWSIM_CHECK(sim, speedAt(sim, 27900) > 0.1, "方块移走后机器人继续前进");
}

FWSIM_SCENARIO(low_battery, "演示：电池电压降到 6.8 V 以下时红色 LED 点亮") {
  demo(sim, "sim/worlds/oval.world");
  sim.setBattery(3000, 6.5);
  sim.runUntil(5000);
  FWSIM_CHECK(sim, sim.levelAt(Robot, LEDR, 2900) == HIGH, "电量充足时红色 LED 熄灭（低电平点亮）");
  FWSIM_CHECK(sim, sim.levelAt(Robot, LEDR, 3600) == LOW, "电压下降后 600 ms 内红色 LED 点亮");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "电量低"), "程序报告电量低");
}

// ---------------------------------------------------- 讲义场地图 ---
namespace {
double closestApproach(const Simulation& sim, Vec2 p) {
  double best = 1e9;
  for (const auto& s : sim.samples()) best = std::min(best, std::hypot(s.pose.x - p.x, s.pose.y - p.y));
  return best;
}
}  // namespace

FWSIM_SCENARIO(course_first_cone, "演示（讲义场地）：在第一条竖段的锥桶前停车") {
  demo(sim, "sim/worlds/course.world");
  sim.gesture(500, GESTURE_UP);
  sim.runUntil(12000);
  FWSIM_CHECK(sim, sim.vehicle().collisions == 0, "机器人始终没有碰到锥桶");
  FWSIM_CHECK(sim, sim.serialContains(Robot, "前方障碍物"), "程序检测到了锥桶");
  FWSIM_CHECK(sim, speedAt(sim, 11900) < 0.01, "机器人停在锥桶前方");
  FWSIM_CHECK(sim, sim.vehicle().pose.y > 0.3, "机器人停车前已沿第一条竖段向上行驶");
}

FWSIM_SCENARIO(course_serpentine, "演示（讲义场地，去掉锥桶）：通过所有 90° 弯到达终点") {
  demo(sim, "sim/worlds/course.world");
  // 纯巡线测试。如果保留边界，示例程序"20 cm 内就停车"的规则会看到
  // 每个底部横段外约 20 cm 处的墙而停下。
  sim.at(0, [](Simulation& s) {
    s.world().circles.clear();
    s.world().walls.clear();
  }, "移除锥桶和边界墙：纯巡线运行");
  sim.gesture(500, GESTURE_UP);
  sim.runUntil(60000);
  const Vec2 end = sim.world().lines.at(0).pts.back();
  const double approach = closestApproach(sim, end);
  double reachedMs = -1;
  for (const auto& s : sim.samples())
    if (std::hypot(s.pose.x - end.x, s.pose.y - end.y) < 0.15) { reachedMs = s.t / 1000.0; break; }
  char buf[160];
  std::snprintf(buf, sizeof buf, "赛道长 %.2f m；离终点最近 %.1f cm；到达时间 %.1f s；"
                "最大偏离 %.1f mm", sim.world().lineLength(), approach * 100, reachedMs / 1000,
                sim.maxLineOffset(1000, reachedMs > 0 ? reachedMs : 60000) * 1000);
  sim.log("检查", buf);
  FWSIM_CHECK(sim, reachedMs > 0, "机器人到达赛道终点（15 cm 以内）");
  FWSIM_CHECK(sim, reachedMs < 0 || sim.maxLineOffset(1000, reachedMs) < 0.06,
              "通过所有弯道时传感器排始终在线中心 6 cm 以内");
  FWSIM_CHECK(sim, sim.vehicle().collisions == 0, "没有碰撞");
}

FWSIM_SCENARIO(course_heat_map, "演示（讲义场地）：沿赛道记录温度并为地图评分") {
  demo(sim, "sim/worlds/course.world");
  // 沿赛道的纯建图运行（为什么对这个示例程序移除边界，见 course_serpentine）。
  sim.at(0, [](Simulation& s) {
    s.world().circles.clear();
    s.world().walls.clear();
  }, "移除锥桶和边界墙：沿赛道建图");
  sim.serialInput(Robot, 100, "pose 0.4425 0.12 90\n");  // 已知的初始位姿
  sim.gesture(2000, GESTURE_UP);                        // 静止 2 s：校准陀螺仪零偏
  sim.gesture(26000, GESTURE_DOWN);
  sim.runUntil(27000);
  const MapScore m = sim.scoreMap();
  char buf[256];
  std::snprintf(buf, sizeof buf,
                "%d 个地图点（%.0f 个唯一点/m^2），覆盖 %.0f%% 的 0.2 m 网格，位置误差平均 %.1f cm / 最大 %.1f cm，"
                "温度误差 %.2f °C，地图误差 %.2f °C",
                m.points, m.pointsPerM2, m.coverage * 100, m.meanPosError * 100, m.maxPosError * 100,
                m.meanTempError, m.meanMapError);
  sim.log("检查", buf);
  double hot = 0, cool = 0;
  int nHot = 0, nCool = 0;
  for (const auto& h : sim.world().heat)
    for (const auto& p : sim.mapPoints())
      if (std::hypot(p.x - h.c.x, p.y - h.c.y) < 0.35) {
        if (h.delta > 0) { hot += p.temp; ++nHot; }
        else { cool += p.temp; ++nCool; }
      }
  const double ambient = sim.world().ambient;
  FWSIM_CHECK(sim, m.points >= 100, "至少记录 100 个温度点");
  FWSIM_CHECK(sim, m.meanPosError < 0.10, "记录的位置与真实位置平均相差 10 cm 以内");
  FWSIM_CHECK(sim, nHot > 0 && hot / nHot > ambient + 3, "地图显示热区比环境温度高");
  FWSIM_CHECK(sim, nCool > 0 && cool / nCool < ambient - 2, "地图显示冷区比环境温度低");
  // 剩余误差大多来自 HS300x 的滞后：tau = 2 s、车速 0.25 m/s 时，
  // 一个读数描述的是沿路径约 0.5 m 之前的空气温度。
  // 补偿这个滞后（例如把读数标记到约 tau 之前的位姿上）可以减小误差。
  FWSIM_CHECK(sim, m.meanMapError < 2.5, "地图温度与真实温度场平均相差 2.5 °C 以内");
}
