#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Mech368Protocol.h>

namespace wire = mech368::ble;
enum class LinkPhase { Retry, Scan, Connect, Discover, Subscribe, Active };
LinkPhase phase = LinkPhase::Retry;
BLEDevice robot;
BLECharacteristic commandCharacteristic;
BLECharacteristic statusCharacteristic;
wire::RemoteLink link;
bool bleReady = false;
bool stopRequested = false;
uint32_t retryAt = 0, activeAt = 0, lastSendAt = 0;

void retry(uint32_t now) {
  link.reset();
  phase = LinkPhase::Retry;
  retryAt = now;
  commandCharacteristic = BLECharacteristic();
  statusCharacteristic = BLECharacteristic();
  if (robot && robot.connected()) robot.disconnect();
  robot = BLEDevice();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200); // 不等待 USB 连接。
  bleReady = BLE.begin();
  if (!bleReady) return;
  // 连接/服务发现/订阅在 ArduinoBLE 内部仍是同步（阻塞）的。
  // 这里限制的是每一次 ATT 等待，而不是一次服务发现的总耗时。
  BLE.setTimeout(100);
}

void loop() {
  const uint32_t beforePoll = millis();
  digitalWrite(LED_BUILTIN, bleReady && link.healthy(beforePoll) ? HIGH :
               ((beforePoll / 125U) % 2U ? HIGH : LOW));
  // 仅用于台架的急停输入。一直"粘住"直到遥控器复位；不假定任何电机引脚。
  // TODO：替换为/增加一个经过明确验证的实体输入，以及 IMU 映射。
  if (Serial.available() > 0 && Serial.read() == 'e') stopRequested = true;
  if (!bleReady) return;
  BLE.poll();
  uint32_t now = millis();
  switch (phase) {
    case LinkPhase::Retry:
      if (static_cast<uint32_t>(now - retryAt) >= 1000) {
        if (BLE.scanForUuid(wire::kService)) phase = LinkPhase::Scan;
        else retryAt = now;
      }
      break;
    case LinkPhase::Scan:
      robot = BLE.available();
      if (robot) { BLE.stopScan(); phase = LinkPhase::Connect; }
      break;
    case LinkPhase::Connect:
      if (robot.connect()) phase = LinkPhase::Discover;
      else retry(millis());
      break;
    case LinkPhase::Discover:
      if (!robot.connected() || !robot.discoverService(wire::kService)) {
        retry(millis()); break;
      }
      commandCharacteristic = robot.characteristic(wire::kCommand);
      statusCharacteristic = robot.characteristic(wire::kStatus);
      if (!commandCharacteristic || !(commandCharacteristic.properties() & BLEWriteWithoutResponse) ||
          !statusCharacteristic || !statusCharacteristic.canSubscribe()) {
        retry(millis()); break;
      }
      phase = LinkPhase::Subscribe;
      break;
    case LinkPhase::Subscribe:
      if (!robot.connected() || !statusCharacteristic.subscribe()) {
        retry(millis()); break;
      }
      // 不读取缓存的 GATT 值，而是等待一条新的状态通知。
      activeAt = millis(); lastSendAt = activeAt;
      phase = LinkPhase::Active;
      break;
    case LinkPhase::Active: {
      if (!robot.connected()) { retry(now); break; }
      bool freshStatus = false;
      if (statusCharacteristic.valueUpdated()) {
        uint8_t bytes[wire::kFrameSize];
        if (statusCharacteristic.valueLength() == static_cast<int>(sizeof(bytes)) &&
            statusCharacteristic.readValue(bytes, sizeof(bytes)) == sizeof(bytes))
          freshStatus = link.status(bytes, sizeof(bytes), millis());
      }
      now = millis(); // 处理状态时本地时钟可能已经前进
      // 450 ms 内应用层 ACK 没有进展即视为故障。启动时留出宽限期，
      // 之后重连；重复的状态永远不能延长健康状态。
      if (static_cast<uint32_t>(now - activeAt) >= mech368::MotionSafety::kTimeoutMs &&
          !link.healthy(now)) { retry(now); break; }
      // 每收到一条有效状态通知（机器人每 50 ms 发一条）就回复一条指令，
      // 这样每条指令都带着最新的挑战码，它的 ACK 也会在下一条状态里回来。
      // 原来独立运行的定时器会漂移到某些相位，使每隔一条指令就过期，
      // 链路因此反复重连（见 docs/SIMULATOR.md）。
      if (link.ready() && freshStatus) {
        lastSendAt = now;
        uint8_t bytes[wire::kFrameSize];
        if (wire::encode(link.command(stopRequested), bytes, sizeof(bytes)) &&
            commandCharacteristic.writeValue(bytes, sizeof(bytes), false)) link.sent();
        else retry(millis());
      }
      break;
    }
  }
}
