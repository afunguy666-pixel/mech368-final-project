#pragma once
// 车与遥控器之间的 BLE 协议。CarMain 和 Remote 两份 protocol.h 必须保持一致。
#include <stdint.h>

#define CAR_SERVICE_UUID "6e8f0001-3c2a-4b8e-9d1f-5a7c2e4b9f10"
#define CAR_CMD_UUID     "6e8f0002-3c2a-4b8e-9d1f-5a7c2e4b9f10"  // 遥控器写
#define CAR_TELEM_UUID   "6e8f0003-3c2a-4b8e-9d1f-5a7c2e4b9f10"  // 车端通知

// 命令：第 1 字节为操作码。任何一条命令都会刷新心跳计时。
enum CmdOp : uint8_t {
  OP_HEARTBEAT  = 0x00,  // 无参数
  OP_ESTOP      = 0x01,  // 急停（任何状态）
  OP_CLEAR      = 0x02,  // 解除急停 → 待机，电机仍不转
  OP_ARM_MANUAL = 0x03,  // 待机 → 遥控驾驶
  OP_ARM_AUTO   = 0x04,  // 待机 → 自主巡线
  OP_DRIVE      = 0x05,  // int8 左轮, int8 右轮，-100..100，仅遥控模式生效
  OP_DISARM     = 0x06,  // 正常停车 → 待机
  OP_CALIBRATE  = 0x07,  // 待机时开始巡线探头校准
};
#define CMD_MAX_LEN 4

enum CarState : uint8_t {
  ST_DISARMED = 0,  // 待机：电机断开，等待授权
  ST_MANUAL,        // 遥控驾驶
  ST_AUTO,          // 自主巡线 + 避障
  ST_ESTOP,         // 急停：需遥控 CLEAR 或本地手势解除
  ST_CALIBRATE,     // 巡线探头校准
  ST_TEST,          // 串口 't'：两轮依次低速转 1 s，检查方向和编码器
};

enum StopReason : uint8_t {
  R_NONE = 0,
  R_REMOTE,        // 遥控急停
  R_LINK_LOST,     // BLE 断开或心跳超时
  R_BUTTON,        // 板上按键
  R_BROWNOUT,      // 锁存器被欠压监测清零（或授权时锁存未能置位）
  R_DRIVER_FAULT,  // DRV8833 nFAULT
  R_LOW_BATTERY,
  R_LINE_LOST,
  R_SERIAL,
};

// 遥测包（小端），车端每 TELEMETRY_MS 通知一次
struct __attribute__((packed)) Telemetry {
  uint8_t  state;     // CarState
  uint8_t  reason;    // StopReason
  uint16_t vbat_mV;   // RAW_AA
  uint8_t  flags;     // TF_*
  int8_t   linePos;   // -100（最左）..100（最右）
  int16_t  speedL;    // 编码器计数/秒
  int16_t  speedR;
  uint8_t  line[5];   // 各路反射强度 >> 4
};

#define TF_LINK_OK    0x01
#define TF_OBS_L      0x02
#define TF_OBS_R      0x04
#define TF_FAULT      0x08
#define TF_LATCH      0x10
#define TF_LINE_FOUND 0x20
#define TF_CALIBRATED 0x40
