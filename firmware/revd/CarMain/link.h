#pragma once
#include "config.h"
#include "protocol.h"

// BLE 外设：接收遥控命令、维持心跳、发送遥测
namespace Link {
  struct Drive { int8_t left = 0; int8_t right = 0; };

  void begin();
  void poll();                        // 每次 loop 调用
  bool connected();
  bool fresh();                       // 已连接且 LINK_TIMEOUT_MS 内收到过命令
  bool popCommand(uint8_t& op);       // 除 DRIVE/HEARTBEAT 外的命令，按到达顺序
  Drive drive();                      // 最近一次 DRIVE；断开后归零
  void sendTelemetry(const Telemetry& t);
}
