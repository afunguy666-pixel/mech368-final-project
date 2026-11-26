#include "link.h"

#if USE_BLE
#include <ArduinoBLE.h>

namespace {
  BLEService svc(CAR_SERVICE_UUID);
  BLECharacteristic cmdChar(CAR_CMD_UUID, BLEWrite | BLEWriteWithoutResponse, CMD_MAX_LEN);
  BLECharacteristic telChar(CAR_TELEM_UUID, BLERead | BLENotify, sizeof(Telemetry));

  bool bleOk = false;
  bool conn = false;
  bool rxSinceConnect = false;
  uint32_t lastRxMs = 0;
  Link::Drive drv;

  // 小环形队列：一次 poll 内收到多条命令（例如急停后紧跟心跳）也不会丢
  uint8_t q[8];
  uint8_t qHead = 0, qTail = 0;
  void push(uint8_t op) {
    uint8_t next = (qHead + 1) & 7;
    if (next != qTail) { q[qHead] = op; qHead = next; }
  }

  // 每次写入都会回调一次（在 BLE.poll() 内）
  void onCmdWritten(BLEDevice, BLECharacteristic c) {
    int n = c.valueLength();
    const uint8_t* v = c.value();
    if (n < 1) return;
    lastRxMs = millis();
    rxSinceConnect = true;
    if (v[0] == OP_DRIVE) {
      if (n >= 3) { drv.left = (int8_t)v[1]; drv.right = (int8_t)v[2]; }
    } else if (v[0] != OP_HEARTBEAT) {
      push(v[0]);
    }
  }
}

void Link::begin() {
  bleOk = BLE.begin();
  if (!bleOk) { Serial.println("BLE init failed"); return; }
  BLE.setLocalName(BLE_NAME);
  BLE.setDeviceName(BLE_NAME);
  BLE.setAdvertisedService(svc);
  svc.addCharacteristic(cmdChar);
  svc.addCharacteristic(telChar);
  BLE.addService(svc);
  cmdChar.setEventHandler(BLEWritten, onCmdWritten);
  BLE.advertise();
  Serial.print("BLE advertising as ");
  Serial.println(BLE_NAME);
}

void Link::poll() {
  if (!bleOk) return;
  BLE.poll();
  bool c = BLE.connected();
  if (c != conn) {
    conn = c;
    rxSinceConnect = false;           // 新连接必须先收到一条命令才算在线
    drv = Drive();
    Serial.println(c ? "BLE connected" : "BLE disconnected");
  }
}

bool Link::connected() { return conn; }

bool Link::fresh() {
  return conn && rxSinceConnect && (millis() - lastRxMs) < LINK_TIMEOUT_MS;
}

bool Link::popCommand(uint8_t& op) {
  if (qTail == qHead) return false;
  op = q[qTail];
  qTail = (qTail + 1) & 7;
  return true;
}

Link::Drive Link::drive() { return fresh() ? drv : Drive(); }

void Link::sendTelemetry(const Telemetry& t) {
  if (conn) telChar.writeValue((const uint8_t*)&t, sizeof(t));
}

#else  // USE_BLE == 0：台架调试用空实现，遥控始终视为离线

void Link::begin() {}
void Link::poll() {}
bool Link::connected() { return false; }
bool Link::fresh() { return false; }
bool Link::popCommand(uint8_t&) { return false; }
Link::Drive Link::drive() { return Drive(); }
void Link::sendTelemetry(const Telemetry&) {}

#endif
