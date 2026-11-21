// MECH 368 双轮巡线车 · 主控固件（Nano 33 BLE Sense Rev2）
// 板卡包：Arduino Mbed OS Nano Boards；库：ArduinoBLE、Arduino_APDS9960
// 硬件和参数见 config.h，协议见 protocol.h，调试步骤见 ../README.md

#include "config.h"
#include "protocol.h"
#include "pid.h"
#include "motors.h"
#include "encoders.h"
#include "sensors.h"
#include "line.h"
#include "link.h"
#include "ui.h"
#if USE_GESTURE
#include <Arduino_APDS9960.h>
#endif

// ───────── 状态 ─────────
static CarState   state  = ST_DISARMED;
static StopReason reason = R_NONE;
static Sensors::Frame frame;

static float    vbatFilt = 0;
#if HAS_VBAT
static uint32_t vbatCritSince = 0;
#endif
static float    speedL = 0, speedR = 0;      // 编码器计数/秒（滤波后）
static int32_t  prevEncL = 0, prevEncR = 0;
static float    lastLinePos = 0;
static uint32_t lineLostSince = 0;
static uint32_t phaseStart = 0;              // 校准 / 测试开始时间
static uint8_t  testPhase = 0;
static bool     printSensors = false;
static bool     gestureOk = false;

static Pid linePid(LINE_KP, LINE_KI, LINE_KD, 0.5f, 1.0f);
static Pid spdPidL(SPEED_KP, SPEED_KI, 0, 0.3f, 0.5f);
static Pid spdPidR(SPEED_KP, SPEED_KI, 0, 0.3f, 0.5f);

static const char* stateName(CarState s) {
  switch (s) {
    case ST_DISARMED:  return "DISARMED";
    case ST_MANUAL:    return "MANUAL";
    case ST_AUTO:      return "AUTO";
    case ST_ESTOP:     return "ESTOP";
    case ST_CALIBRATE: return "CALIBRATE";
    case ST_TEST:      return "TEST";
  }
  return "?";
}

static const char* reasonName(StopReason r) {
  switch (r) {
    case R_NONE:         return "-";
    case R_REMOTE:       return "remote e-stop";
    case R_LINK_LOST:    return "BLE link lost";
    case R_BUTTON:       return "button";
    case R_BROWNOUT:     return "latch cleared / VSW low";
    case R_DRIVER_FAULT: return "DRV8833 fault";
    case R_LOW_BATTERY:  return "battery critical";
    case R_LINE_LOST:    return "line lost";
    case R_SERIAL:       return "serial";
  }
  return "?";
}

static bool isRunning() { return state == ST_MANUAL || state == ST_AUTO || state == ST_TEST; }

// ───────── 状态切换 ─────────
static void enterEstop(StopReason r) {
  Motors::disarm();
  if (state != ST_ESTOP || reason != r) {
    Serial.print("[ESTOP] ");
    Serial.println(reasonName(r));
  }
  state = ST_ESTOP;
  reason = r;
}

static void enterDisarmed(StopReason r) {
  Motors::disarm();
  state = ST_DISARMED;
  reason = r;
  Serial.print("[DISARMED] ");
  Serial.println(reasonName(r));
}

static void resetControllers() {
  linePid.reset();
  spdPidL.reset();
  spdPidR.reset();
  lineLostSince = 0;
}

static void tryArm(CarState target) {
  if (state != ST_DISARMED) {
    Serial.println("arm refused: not DISARMED (clear e-stop first)");
    return;
  }
  bool needLink = (target == ST_MANUAL) || (target == ST_AUTO && REQUIRE_LINK_IN_AUTO);
  if (needLink && !Link::fresh()) { Serial.println("arm refused: remote not linked"); return; }
#if HAS_FAULT
  if (frame.fault) { Serial.println("arm refused: driver fault"); return; }
#endif
#if HAS_VBAT
  if (vbatFilt > 0.5f && vbatFilt < VBAT_CRIT_V) { Serial.println("arm refused: battery critical"); return; }
#endif
  if (target == ST_AUTO && !Line::isCalibrated()) Serial.println("warning: line sensors not calibrated");

  resetControllers();
  if (!Motors::arm()) {
    enterEstop(R_BROWNOUT);
    Serial.println("latch did not set: battery off or VSW below threshold?");
    return;
  }
  state = target;
  reason = R_NONE;
  phaseStart = millis();
  testPhase = 0;
  Serial.print("[ARMED] ");
  Serial.println(stateName(state));
}

static void startCalibration() {
  if (state != ST_DISARMED) return;
  Line::resetCalibration();
  state = ST_CALIBRATE;
  phaseStart = millis();
  Serial.println("[CALIBRATE] sweep every line sensor across line and background...");
}

static void handleOp(uint8_t op, StopReason estopReason) {
  switch (op) {
    case OP_ESTOP:      enterEstop(estopReason); break;
    case OP_CLEAR:      if (state == ST_ESTOP) enterDisarmed(R_NONE); break;
    case OP_DISARM:     if (isRunning()) enterDisarmed(R_NONE); break;
    case OP_ARM_MANUAL: tryArm(ST_MANUAL); break;
    case OP_ARM_AUTO:   tryArm(ST_AUTO); break;
    case OP_CALIBRATE:  startCalibration(); break;
    default: break;
  }
}

// ───────── 运动 ─────────
// 轮速环：目标为 -1..1（相对 MAX_WHEEL_CPS），输出 PWM 占空比
static float wheelOut(float target, float measuredCps, Pid& pid, float dt) {
#if USE_SPEED_LOOP
  if (fabsf(target) < 0.01f) { pid.reset(); return 0; }
  float err = (target * MAX_WHEEL_CPS - measuredCps) / MAX_WHEEL_CPS;
  return constrain(target + pid.update(err, dt), -1.0f, 1.0f);
#else
  (void)measuredCps; (void)pid; (void)dt;
  return target;
#endif
}

// 避障策略：按赛道规则修改。默认停车等待，障碍移开后继续巡线。
static void obstacleStep(float& uL, float& uR) {
  uL = uR = 0;
  linePid.reset();
}

static void autoStep(float dt, float& uL, float& uR) {
  if (frame.obstacleL || frame.obstacleR) { obstacleStep(uL, uR); return; }

  float pos;
  if (Line::position(frame.line, pos)) {
    lineLostSince = 0;
    lastLinePos = pos;
    float steer = linePid.update(pos, dt);           // 线在左（pos<0）→ 左轮慢、右轮快
    float base = AUTO_BASE_SPEED * (1.0f - 0.5f * fabsf(pos));  // 偏离越大越减速
    uL = base + steer;
    uR = base - steer;
  } else {
    if (lineLostSince == 0) lineLostSince = millis();
    if (millis() - lineLostSince > LOST_TIMEOUT_MS) { enterDisarmed(R_LINE_LOST); return; }
    float s = (lastLinePos < 0) ? -LOST_SEARCH_SPEED : LOST_SEARCH_SPEED;
    uL = s;                                          // 朝最后看到线的一侧原地转
    uR = -s;
  }
}

static void testStep(float& uL, float& uR) {
  uint32_t t = millis() - phaseStart;
  if (t < 1000)      uL = 0.3f;
  else if (t < 2000) uR = 0.3f;
  if (t >= 900 && testPhase == 0) {
    testPhase = 1;
    Serial.print("TEST left  fwd 30%: speedL="); Serial.print(speedL);
    Serial.print(" cps, speedR="); Serial.println(speedR);
  }
  if (t >= 1900 && testPhase == 1) {
    testPhase = 2;
    Serial.print("TEST right fwd 30%: speedL="); Serial.print(speedL);
    Serial.print(" cps, speedR="); Serial.println(speedR);
    Serial.println("  左/右轮应向前转，对应计数为正；否则改 config.h 的 *_INVERT");
  }
  if (t >= 2000) enterDisarmed(R_NONE);
}

// ───────── 100 Hz 控制周期 ─────────
static void controlTick() {
  const float dt = 1.0f / CONTROL_HZ;
  Sensors::scan(frame);

#if HAS_VBAT
  vbatFilt = (vbatFilt == 0) ? frame.vbat : vbatFilt + 0.05f * (frame.vbat - vbatFilt);
#endif

  int32_t l, r;
  Encoders::snapshot(l, r);
  speedL += 0.3f * ((l - prevEncL) / dt - speedL);
  speedR += 0.3f * ((r - prevEncR) / dt - speedR);
  prevEncL = l;
  prevEncR = r;

  // 安全检查：任何一项不满足都立即断开电机
  if (isRunning()) {
#if HAS_LATCH
    if (!frame.latch) enterEstop(R_BROWNOUT);
#endif
#if HAS_FAULT
    if (frame.fault) enterEstop(R_DRIVER_FAULT);
#endif
    bool needLink = (state == ST_MANUAL) || (state == ST_AUTO && REQUIRE_LINK_IN_AUTO);
    if (isRunning() && needLink && !Link::fresh()) enterEstop(R_LINK_LOST);
  }
#if HAS_VBAT
  if (vbatFilt > 0.5f && vbatFilt < VBAT_CRIT_V) {
    if (vbatCritSince == 0) vbatCritSince = millis();
    if (isRunning() && millis() - vbatCritSince > VBAT_CRIT_MS) enterEstop(R_LOW_BATTERY);
  } else {
    vbatCritSince = 0;
  }
#endif

  float uL = 0, uR = 0;
  switch (state) {
    case ST_MANUAL: {
      Link::Drive d = Link::drive();
      uL = d.left / 100.0f * MANUAL_MAX;
      uR = d.right / 100.0f * MANUAL_MAX;
      break;
    }
    case ST_AUTO:
      autoStep(dt, uL, uR);
      break;
    case ST_TEST:
      testStep(uL, uR);
      break;
    case ST_CALIBRATE:
      Line::addCalibrationSample(frame.line);
      if (millis() - phaseStart > CALIBRATE_MS) {
        Line::printCalibration(Serial);
        enterDisarmed(R_NONE);
      }
      break;
    default:
      break;
  }

  if (state == ST_MANUAL || state == ST_AUTO) {
    Motors::set(wheelOut(uL, speedL, spdPidL, dt), wheelOut(uR, speedR, spdPidR, dt));
  } else if (state == ST_TEST) {
    Motors::set(uL, uR);                             // 测试用开环
  } else {
    Motors::set(0, 0);
  }
}

// ───────── 输入 ─────────
static void printHelp() {
  Serial.println(F("MECH368 CarMain  serial: x=estop c=clear m=manual a=auto d=disarm "
                   "k=calibrate t=wheel test s=toggle print h=help"));
}

static void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'x': case ' ': handleOp(OP_ESTOP, R_SERIAL); break;
      case 'c': handleOp(OP_CLEAR, R_SERIAL); break;
      case 'm': handleOp(OP_ARM_MANUAL, R_SERIAL); break;
      case 'a': handleOp(OP_ARM_AUTO, R_SERIAL); break;
      case 'd': handleOp(OP_DISARM, R_SERIAL); break;
      case 'k': handleOp(OP_CALIBRATE, R_SERIAL); break;
      case 't': tryArm(ST_TEST); break;               // 架空车轮再测！
      case 's': printSensors = !printSensors; break;
      case 'h': case '?': printHelp(); break;
      default: break;
    }
  }
}

static void handleButton() {
  switch (UI::button()) {
    case UI::PRESS_DOWN:                             // 运行中按下 = 本地急停
      if (isRunning()) enterEstop(R_BUTTON);
      break;
    case UI::PRESS_LONG:                             // 待机时长按 = 巡线校准
      if (state == ST_DISARMED) startCalibration();
      break;
    default:
      break;
  }
}

static void handleGesture() {
#if USE_GESTURE
  if (!gestureOk || (state != ST_ESTOP && state != ST_DISARMED)) return;
  static uint32_t last = 0;
  if (millis() - last < GESTURE_POLL_MS) return;
  last = millis();
  if (!APDS.gestureAvailable()) return;
  int g = APDS.readGesture();
  if (g == GESTURE_UP && state == ST_ESTOP) {        // 向上挥手 = 本地解除急停
    Serial.println("gesture UP: clear e-stop");
    enterDisarmed(R_NONE);
  } else if (g == GESTURE_RIGHT && state == ST_DISARMED) {  // 向右挥手 = 启动自主
    Serial.println("gesture RIGHT: arm AUTO");
    tryArm(ST_AUTO);
  }
#endif
}

// ───────── 输出 ─────────
static void updateIndicators() {
  using namespace UI;
  if (Link::fresh())                                         set(LED_LINK, P_ON);
  else if (Link::connected() || reason == R_LINK_LOST)       set(LED_LINK, P_FAST);
  else                                                       set(LED_LINK, P_OFF);

  if (state == ST_ESTOP)          set(LED_ESTOP, reason == R_LINK_LOST ? P_FAST : P_ON);
  else if (state == ST_CALIBRATE) set(LED_ESTOP, P_SLOW);
  else                            set(LED_ESTOP, P_OFF);

  if (vbatFilt > 0.5f && vbatFilt < VBAT_CRIT_V)     set(LED_LOW, P_FAST);
  else if (vbatFilt > 0.5f && vbatFilt < VBAT_LOW_V) set(LED_LOW, P_ON);
  else                                               set(LED_LOW, P_OFF);

  // 板载 RGB：绿 = 遥控，蓝 = 自主，红 = 急停，黄 = 校准/测试，灭 = 待机
  switch (state) {
    case ST_MANUAL:    rgb(false, true, false); break;
    case ST_AUTO:      rgb(false, false, true); break;
    case ST_ESTOP:     rgb(true, false, false); break;
    case ST_CALIBRATE:
    case ST_TEST:      rgb(true, true, false); break;
    default:           rgb(false, false, false); break;
  }
  UI::update();
}

static void sendTelemetry() {
  Telemetry t = {};
  t.state = state;
  t.reason = reason;
  t.vbat_mV = (uint16_t)(vbatFilt * 1000.0f);
  float pos = 0;
  bool found = Line::position(frame.line, pos);
  t.flags = (Link::fresh() ? TF_LINK_OK : 0) | (frame.obstacleL ? TF_OBS_L : 0) |
            (frame.obstacleR ? TF_OBS_R : 0) | (frame.fault ? TF_FAULT : 0) |
            (frame.latch ? TF_LATCH : 0) | (found ? TF_LINE_FOUND : 0) |
            (Line::isCalibrated() ? TF_CALIBRATED : 0);
  t.linePos = found ? (int8_t)(pos * 100) : 0;
  t.speedL = (int16_t)constrain(speedL, -32767.0f, 32767.0f);
  t.speedR = (int16_t)constrain(speedR, -32767.0f, 32767.0f);
  for (uint8_t i = 0; i < 5; i++) t.line[i] = (i < NUM_LINE) ? frame.line[i] >> 4 : 0;
  Link::sendTelemetry(t);
}

static void debugPrint() {
  Serial.print(stateName(state));
  Serial.print(" vbat="); Serial.print(vbatFilt, 2);
  Serial.print(" line=");
  for (uint8_t i = 0; i < NUM_LINE; i++) { Serial.print(frame.line[i]); Serial.print(' '); }
  float pos;
  if (Line::position(frame.line, pos)) { Serial.print("pos="); Serial.print(pos, 2); }
  else Serial.print("pos=--");
  Serial.print(" obs="); Serial.print(frame.obstacleL); Serial.print(frame.obstacleR);
  Serial.print('('); Serial.print(frame.obsRaw[0]); Serial.print(','); Serial.print(frame.obsRaw[1]); Serial.print(')');
  Serial.print(" enc="); Serial.print(prevEncL); Serial.print(','); Serial.print(prevEncR);
  Serial.print(" cps="); Serial.print(speedL, 0); Serial.print(','); Serial.print(speedR, 0);
  Serial.print(" latch="); Serial.print(frame.latch);
  Serial.print(" fault="); Serial.print(frame.fault);
  Serial.print(" link="); Serial.println(Link::fresh());
}

// ───────── 入口 ─────────
void setup() {
  Motors::begin();            // 第一件事：ARM 拉低，电机保持断开
  Serial.begin(115200);       // 不等待串口，脱离电脑也能运行
  Sensors::begin();
  Encoders::begin();
  UI::begin();
  Line::resetCalibration();
  Link::begin();
#if USE_GESTURE
  gestureOk = APDS.begin();
  if (!gestureOk) Serial.println("APDS9960 init failed: gesture disabled");
#endif
  printHelp();
}

void loop() {
  Link::poll();
  uint8_t op;
  while (Link::popCommand(op)) handleOp(op, R_REMOTE);
  handleSerial();
  handleButton();
  handleGesture();

  static uint32_t nextTick = micros();
  if ((int32_t)(micros() - nextTick) >= 0) {
    nextTick += 1000000UL / CONTROL_HZ;
    if ((int32_t)(micros() - nextTick) > 50000) nextTick = micros();  // 落后太多时重新对齐
    controlTick();
  }

  updateIndicators();

  static uint32_t lastTel = 0, lastDbg = 0;
  if (millis() - lastTel >= TELEMETRY_MS) { lastTel = millis(); sendTelemetry(); }
  if (printSensors && millis() - lastDbg >= DEBUG_PRINT_MS) { lastDbg = millis(); debugPrint(); }
}
