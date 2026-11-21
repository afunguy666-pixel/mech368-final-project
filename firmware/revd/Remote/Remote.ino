// MECH 368 手持遥控器（BLE 中心设备）
// 适用于任何支持 ArduinoBLE 的板：Nano 33 BLE / BLE Sense / Nano 33 IoT / Nano ESP32 等。
// Uno、Nano（ATmega328）没有蓝牙，不能用。
//
// 按键均接 GND（内部上拉）：
//   D2 急停：按下立即发送 ESTOP（大红按钮）
//   D3 启动：短按 = 遥控驾驶，长按 1 s = 自主巡线
//   D4 恢复：短按 = 解除急停，长按 1 s = 正常停车（DISARM）
// 摇杆（可选）：X → A0，Y → A1，3V3 供电
// 无论是否有摇杆，每 SEND_MS 都会发一条命令作为心跳；遥控器断电或离开范围，车在 500 ms 内急停。

#include <ArduinoBLE.h>
#include "protocol.h"

#define CAR_NAME      "MECH368-CAR-01"   // 与车端 config.h 的 BLE_NAME 一致
#define PIN_ESTOP     2
#define PIN_START     3
#define PIN_RECOVER   4
#define USE_JOYSTICK  1
#define PIN_JOY_X     A0
#define PIN_JOY_Y     A1
#define JOY_DEADZONE  8                  // -100..100 中的死区
#define SEND_MS       50                 // 心跳 / 驾驶命令周期，远小于车端 500 ms 超时
#define LONG_MS       1000

BLEDevice car;
BLECharacteristic cmdChar;
BLECharacteristic telChar;
int joyX0 = 512, joyY0 = 512;

struct Button {
  uint8_t pin;
  bool stable = HIGH, last = HIGH, longFired = false;
  uint32_t tChange = 0, tDown = 0;
  explicit Button(uint8_t p) : pin(p) {}
  // 返回：0 无，1 按下，2 短按松开，3 长按
  uint8_t poll() {
    uint32_t now = millis();
    bool r = digitalRead(pin);
    if (r != last) { last = r; tChange = now; }
    if (now - tChange > 20 && r != stable) {
      stable = r;
      if (stable == LOW) { tDown = now; longFired = false; return 1; }
      if (!longFired) return 2;
    }
    if (stable == LOW && !longFired && now - tDown >= LONG_MS) { longFired = true; return 3; }
    return 0;
  }
};
Button btnEstop(PIN_ESTOP), btnStart(PIN_START), btnRecover(PIN_RECOVER);

bool send(uint8_t op, int8_t a = 0, int8_t b = 0, bool withResponse = false) {
  uint8_t buf[3] = {op, (uint8_t)a, (uint8_t)b};
  return cmdChar.writeValue(buf, op == OP_DRIVE ? 3 : 1, withResponse);
}

bool connectCar() {
  Serial.print("scanning for ");
  Serial.println(CAR_NAME);
  BLE.scanForName(CAR_NAME);
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    digitalWrite(LED_BUILTIN, (millis() / 150) & 1);
    BLEDevice d = BLE.available();
    if (!d) continue;
    BLE.stopScan();
    if (!d.connect()) { Serial.println("connect failed"); return false; }
    if (!d.discoverAttributes()) { Serial.println("discover failed"); d.disconnect(); return false; }
    cmdChar = d.characteristic(CAR_CMD_UUID);
    telChar = d.characteristic(CAR_TELEM_UUID);
    if (!cmdChar) { Serial.println("command characteristic missing"); d.disconnect(); return false; }
    if (telChar && telChar.canSubscribe()) telChar.subscribe();
    car = d;
    send(OP_HEARTBEAT);
    Serial.println("connected");
    return true;
  }
  BLE.stopScan();
  return false;
}

int8_t readAxis(uint8_t pin, int center) {
  int v = (analogRead(pin) - center) * 100 / 512;
  if (abs(v) < JOY_DEADZONE) v = 0;
  return (int8_t)constrain(v, -100, 100);
}

void printTelemetry(const Telemetry& t) {
  static const char* names[] = {"DISARMED", "MANUAL", "AUTO", "ESTOP", "CALIBRATE", "TEST"};
  Serial.print(t.state < 6 ? names[t.state] : "?");
  Serial.print(" reason="); Serial.print(t.reason);
  Serial.print(" vbat="); Serial.print(t.vbat_mV / 1000.0f, 2);
  Serial.print(" pos="); Serial.print(t.linePos);
  Serial.print(" obs="); Serial.print((t.flags & TF_OBS_L) ? 'L' : '-'); Serial.print((t.flags & TF_OBS_R) ? 'R' : '-');
  Serial.print(" cps="); Serial.print(t.speedL); Serial.print(','); Serial.println(t.speedR);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_RECOVER, INPUT_PULLUP);
#if USE_JOYSTICK
  delay(50);
  joyX0 = analogRead(PIN_JOY_X);     // 上电时摇杆须居中
  joyY0 = analogRead(PIN_JOY_Y);
#endif
  if (!BLE.begin()) {
    Serial.println("BLE init failed");
    while (true) { digitalWrite(LED_BUILTIN, (millis() / 50) & 1); }
  }
}

void loop() {
  if (!car || !car.connected()) {
    connectCar();
    return;
  }
  digitalWrite(LED_BUILTIN, HIGH);
  BLE.poll();

  if (btnEstop.poll() == 1) {
    for (int i = 0; i < 3; i++) send(OP_ESTOP, 0, 0, true);  // 带应答，重复三次
    Serial.println(">> ESTOP");
  }
  switch (btnStart.poll()) {
    case 2: send(OP_ARM_MANUAL, 0, 0, true); Serial.println(">> ARM MANUAL"); break;
    case 3: send(OP_ARM_AUTO, 0, 0, true);   Serial.println(">> ARM AUTO");   break;
  }
  switch (btnRecover.poll()) {
    case 2: send(OP_CLEAR, 0, 0, true);  Serial.println(">> CLEAR");  break;
    case 3: send(OP_DISARM, 0, 0, true); Serial.println(">> DISARM"); break;
  }

  static uint32_t lastSend = 0;
  if (millis() - lastSend >= SEND_MS) {
    lastSend = millis();
#if USE_JOYSTICK
    int8_t x = readAxis(PIN_JOY_X, joyX0);
    int8_t y = readAxis(PIN_JOY_Y, joyY0);
    int l = constrain(y + x, -100, 100);   // 差速混控：Y 前后，X 转向
    int r = constrain(y - x, -100, 100);
    send(OP_DRIVE, (int8_t)l, (int8_t)r);
#else
    send(OP_HEARTBEAT);
#endif
  }

  static uint32_t lastPrint = 0;
  if (telChar && telChar.valueUpdated() && millis() - lastPrint > 500) {
    lastPrint = millis();
    Telemetry t;
    telChar.readValue(&t, sizeof(t));
    printTelemetry(t);
  }
}
