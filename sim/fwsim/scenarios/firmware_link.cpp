// 针对真实固件组合 firmware/robot + firmware/remote 的场景。
// 和台架仪器一样，只从外部观察固件：LED 引脚、串口输出，
// 以及（模拟的）空中传输的 BLE 报文。
#include "Simulation.h"
#include <cstdio>

using namespace fwsim;
namespace wire = mech368::ble;

namespace {
const double kWatchdogMs = mech368::MotionSafety::kTimeoutMs;

void boot(Simulation& sim, double remoteOnMs = 300, double robotOnMs = 0) {
  sim.loadWorld("sim/worlds/course.world");
  sim.loadVehicle("sim/vehicles/example.vehicle");
  sim.useSketch(Robot, "robot", robotOnMs);
  sim.useSketch(Remote, "remote", remoteOnMs);
}

bool flag(const FrameRecord* f, int bit) { return f && (f->frame.linear & bit) != 0; }

// 机器人第一次报告允许运动的时间（ms），没有则为 -1。
double firstPermitted(const Simulation& sim, double fromMs = 0) {
  for (const auto& f : sim.frames())
    if (!f.fromRemote && f.decoded && f.frame.type == wire::Type::Status &&
        f.sentUs >= fromMs * 1000 && (f.frame.linear & wire::MotionPermitted))
      return f.sentUs / 1000.0;
  return -1;
}

// 在 [from, to] 区间内不能有任何状态报告"允许运动"。
bool neverPermitted(const Simulation& sim, double fromMs, double toMs) {
  for (const auto& f : sim.frames())
    if (!f.fromRemote && f.decoded && f.frame.type == wire::Type::Status &&
        f.sentUs >= fromMs * 1000 && f.sentUs <= toMs * 1000 && (f.frame.linear & wire::MotionPermitted))
      return false;
  return true;
}

// 每次运行都要满足的安全不变式：只要机器人报告允许运动，
// 在此之前不到一个看门狗周期内，必须有一条运动指令送达过它。
void checkWatchdogInvariant(Simulation& sim) {
  int violations = 0;
  double worstMs = 0;
  for (const auto& status : sim.frames()) {
    if (status.fromRemote || !status.decoded || status.frame.type != wire::Type::Status ||
        !(status.frame.linear & wire::MotionPermitted))
      continue;
    double latest = -1;
    for (const auto& cmd : sim.frames())
      if (cmd.fromRemote && cmd.fate == RadioPacket::Delivered && cmd.decoded &&
          cmd.frame.type == wire::Type::Motion && cmd.deliverUs <= status.sentUs)
        latest = std::max(latest, cmd.deliverUs / 1000.0);
    const double age = latest < 0 ? 1e9 : status.sentUs / 1000.0 - latest;
    worstMs = std::max(worstMs, age);
    if (age > kWatchdogMs + 5) ++violations;  // 5 ms：送达后机器人要等下一次 poll 才处理
  }
  char buf[160];
  std::snprintf(buf, sizeof buf,
                "看门狗不变式：每条\"允许运动\"的状态都在一条已送达的运动指令之后 < %.0f ms"
                "（最差 %.1f ms）", kWatchdogMs, worstMs);
  FWSIM_CHECK(sim, violations == 0, buf);
}

void checkHealthy(Simulation& sim, double fromMs, double toMs, const char* when) {
  FWSIM_CHECK(sim, sim.steady(Remote, LED_BUILTIN, HIGH, fromMs, toMs),
              std::string(when) + "：遥控器 LED 常亮（链路健康）");
  FWSIM_CHECK(sim, sim.steady(Robot, LED_BUILTIN, HIGH, fromMs, toMs),
              std::string(when) + "：机器人 LED 常亮（有链路且允许运动）");
  const FrameRecord* s = sim.statusSentBefore(toMs);
  FWSIM_CHECK(sim, flag(s, wire::AckValid) && flag(s, wire::MotionPermitted) &&
                       !flag(s, wire::EstopLatched) && flag(s, wire::OutputsDisabled),
              std::string(when) + "：状态标志为 ACK + 允许运动、无急停、输出已禁用");
}
}  // namespace

FWSIM_SCENARIO(link_boot, "机器人先启动，遥控器 300 ms 后上电；链路变为健康") {
  boot(sim);
  sim.runUntil(5000);
  const double up = firstPermitted(sim);
  sim.log("检查", "第一条允许运动的状态发送于 " + std::to_string(up) + " ms");
  FWSIM_CHECK(sim, up > 0 && up < 3000, "上电后 3 s 内建立链路和心跳");
  FWSIM_CHECK(sim, sim.blinking(Remote, LED_BUILTIN, 300, 1200), "搜索期间遥控器 LED 闪烁");
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, 0, 1200), "建立链路之前机器人 LED 闪烁");
  checkHealthy(sim, 3500, 5000, "启动后");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(link_boot_robot_late, "遥控器先上电；机器人 2.5 s 后才上电") {
  boot(sim, 0, 2500);
  sim.runUntil(8000);
  const double up = firstPermitted(sim);
  FWSIM_CHECK(sim, up > 2500 && up < 6000, "机器人上电后 3.5 s 内建立链路");
  FWSIM_CHECK(sim, sim.blinking(Remote, LED_BUILTIN, 0, 2500), "机器人未上电时遥控器 LED 闪烁");
  checkHealthy(sim, 6500, 8000, "机器人延迟启动后");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(command_silence, "链路保持连接，但遥控器->机器人的指令中断 2 s") {
  boot(sim);
  sim.block(true, 4000, 6000);
  sim.runUntil(10000);
  // 4 s 时已在途中的报文仍会到达，所以要看整个时间窗口。
  const double last = sim.lastCommandDelivered(6000);
  char buf[96];
  std::snprintf(buf, sizeof buf, "最后一条指令送达于 %.1f ms", last);
  sim.log("检查", buf);
  FWSIM_CHECK(sim, neverPermitted(sim, last + kWatchdogMs + 5, 6000),
              "机器人在 450 ms 看门狗时间内停止报告\"允许运动\"");
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, last + kWatchdogMs, 6000),
              "收不到指令时机器人 LED 闪烁");
  FWSIM_CHECK(sim, sim.blinking(Remote, LED_BUILTIN, 4000 + kWatchdogMs + 100, 6000),
              "指令不再被确认时遥控器 LED 闪烁");
  checkHealthy(sim, 8500, 10000, "指令恢复后");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(status_silence, "链路保持连接，但机器人->遥控器的状态中断 2 s") {
  boot(sim);
  sim.block(false, 4000, 6000);
  sim.runUntil(10000);
  FWSIM_CHECK(sim, sim.blinking(Remote, LED_BUILTIN, 4000 + kWatchdogMs + 100, 6000),
              "收不到状态/ACK 时遥控器 LED 闪烁");
  checkHealthy(sim, 8500, 10000, "状态恢复后");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(link_loss, "无线链路丢失（超出范围）3 s 后恢复") {
  boot(sim);
  sim.linkLoss(4000, 7000);
  sim.runUntil(12000);
  const double last = sim.lastCommandDelivered(7000);
  FWSIM_CHECK(sim, sim.levelAt(Robot, LED_BUILTIN, last + kWatchdogMs + 130) == LOW ||
                       sim.transitions(Robot, LED_BUILTIN, last, last + kWatchdogMs + 260) > 0,
              "机器人 LED 在看门狗时间 + 一个闪烁周期内脱离\"常亮\"");
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, 4500, 7000), "断链期间机器人 LED 闪烁");
  FWSIM_CHECK(sim, sim.blinking(Remote, LED_BUILTIN, 4600, 7000), "断链期间遥控器 LED 闪烁");
  checkHealthy(sim, 10500, 12000, "链路恢复后");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(remote_power_off, "连接中遥控器突然断电；机器人必须自行停止") {
  boot(sim);
  sim.powerOff(Remote, 4000);
  sim.runUntil(8000);
  const double last = sim.lastCommandDelivered(8000);
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, last + kWatchdogMs, 8000),
              "遥控器消失后机器人 LED 闪烁");
  FWSIM_CHECK(sim, sim.hasEvent("关闭了连接") || sim.hasEvent("丢失"),
              "机器人一侧最终察觉到断开");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(estop_latch, "遥控器急停（串口输入 'e'）在断链后仍保持锁存") {
  boot(sim);
  sim.serialInput(Remote, 4000, "e");
  sim.linkLoss(6000, 7000);
  sim.runUntil(11000);
  double latchedAt = -1;
  for (const auto& f : sim.frames())
    if (!f.fromRemote && f.decoded && f.frame.type == wire::Type::Status && f.sentUs >= 4000000 &&
        (f.frame.linear & wire::EstopLatched)) {
      latchedAt = f.sentUs / 1000.0;
      break;
    }
  FWSIM_CHECK(sim, latchedAt > 0 && latchedAt < 4200, "机器人在 200 ms 内报告急停已锁存");
  FWSIM_CHECK(sim, neverPermitted(sim, latchedAt, 11000), "急停之后不再允许运动");
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, 4300, 6000), "锁存期间机器人 LED 闪烁");
  FWSIM_CHECK(sim, flag(sim.statusSentBefore(11000), wire::EstopLatched),
              "锁存在断链和重连后仍然保持");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(corrupt_commands, "1 s 内每条遥控器->机器人的报文都有一个比特翻转") {
  boot(sim);
  sim.at(4000, [](Simulation& s) { s.radio().corruptCentralToPeripheral = 1.0; },
         "无线：损坏每一条遥控器 -> 机器人的报文");
  sim.at(5000, [](Simulation& s) { s.radio().corruptCentralToPeripheral = 0.0; },
         "无线：停止损坏报文");
  sim.runUntil(9000);
  const double last = sim.lastCommandDelivered(5000);  // 损坏的报文不计入
  FWSIM_CHECK(sim, neverPermitted(sim, last + kWatchdogMs + 5, 5000),
              "损坏的报文永远不能延长看门狗");
  checkHealthy(sim, 7500, 9000, "停止损坏后");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(lossy_link, "双向 10% 随机丢包，持续 10 s") {
  boot(sim);
  sim.at(2000, [](Simulation& s) {
    s.radio().lossCentralToPeripheral = 0.10;
    s.radio().lossPeripheralToCentral = 0.10;
  }, "无线：双向 10% 丢包");
  sim.at(12000, [](Simulation& s) {
    s.radio().lossCentralToPeripheral = 0;
    s.radio().lossPeripheralToCentral = 0;
  }, "无线：关闭丢包");
  sim.runUntil(15000);
  int reconnects = 0;
  for (const auto& e : sim.events())
    if (e.t >= 3000000 && e.t <= 12000000 && e.text.find("关闭了连接") != std::string::npos) ++reconnects;
  sim.log("检查", "丢包期间遥控器主动重连的次数：" + std::to_string(reconnects));
  FWSIM_CHECK(sim, reconnects <= 2, "偶尔丢包不会让遥控器反复重连");
  checkHealthy(sim, 13500, 15000, "停止丢包后");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(millis_rollover, "链路连接期间，两块板的 millis() 越过 2^32 ms 回绕") {
  boot(sim);
  // 让两块板的时钟都从 millis() 溢出前 4 s 开始（相当于已运行 49.7 天）。
  const uint64_t wrap = (uint64_t(1) << 32) * 1000ULL - 4000000ULL;
  sim.board(Robot).timeOffsetUs = wrap;
  sim.board(Remote).timeOffsetUs = wrap;
  sim.runUntil(9000);
  checkHealthy(sim, 3000, 9000, "在约 4 s 处 millis() 回绕前后");
  checkWatchdogInvariant(sim);
}

FWSIM_SCENARIO(link_stability, "理想无线条件下运行 60 s：链路一旦健康，就不能再断开") {
  boot(sim);
  sim.runUntil(60000);
  const double up = firstPermitted(sim);
  int drops = 0;
  double firstDrop = -1;
  for (const auto& e : sim.events())
    if (e.t > up * 1000 && e.text.find("关闭了连接") != std::string::npos) {
      if (firstDrop < 0) firstDrop = e.t / 1000.0;
      ++drops;
    }
  char buf[160];
  std::snprintf(buf, sizeof buf, "链路在 %.0f ms 变为健康；之后遥控器主动重连 %d 次%s",
                up, drops, drops ? "（可在回放中查看第一次）" : "");
  sim.log("检查", buf);
  FWSIM_CHECK(sim, up > 0, "链路变为健康");
  FWSIM_CHECK(sim, drops == 0, "理想无线条件下，首次健康之后没有任何重连");
  checkWatchdogInvariant(sim);
}
