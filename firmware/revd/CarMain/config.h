#pragma once
// MECH 368 双轮巡线车 · 主控固件配置
// 引脚和可调参数集中在这里。换板只改 BOARD；标 TODO 的数值要按实车测量后填。

#include <Arduino.h>

// ───────────────────────── 1. 选择硬件 ─────────────────────────
#define BOARD_REVD       1   // RevD PCB（5 路巡线、多路器、锁存器）
#define BOARD_PERFBOARD  2   // 洞洞板版（3 路巡线直连 A0–A2，SLP 直接由 D10 控制）
#ifndef BOARD
#define BOARD BOARD_REVD
#endif

#ifndef USE_BLE
#define USE_BLE 1            // 0 = 不编译蓝牙，只用 USB 串口台架调试
#endif
#ifndef USE_GESTURE
#define USE_GESTURE 1        // 板载 APDS9960 手势：本地解除急停 / 启动自主
#endif

// ───────────────────────── 2. 引脚 ─────────────────────────
#if BOARD == BOARD_REVD
  // 电机（DRV8833 #3297）：每轮 IN1 输出 PWM，IN2 作方向
  #define PIN_L_IN1     6    // LP
  #define PIN_L_IN2     7    // LD
  #define PIN_R_IN1     9    // RP
  #define PIN_R_IN2     8    // RD
  #define PIN_ARM       10   // 电机许可：上升沿置位锁存；低电平立即停机
  #define PIN_IREN      11   // Q1：IR 发射管总开关，高 = 亮
  #define PIN_BTN       12   // SW2 MODE，按下接地
  #define PIN_LED_LOW   13   // D2 LOW BAT
  #define PIN_LED_LINK  1    // D3 LINK（Nano TX 脚）
  #define PIN_LED_ESTOP 0    // D4 ESTOP（Nano RX 脚）
  #define PIN_ENC_LA    2
  #define PIN_ENC_LB    3
  #define PIN_ENC_RA    4
  #define PIN_ENC_RB    5
  #define PIN_MUX_S0    A0
  #define PIN_MUX_S1    A1
  #define PIN_MUX_S2    A2
  #define PIN_MUX_Z     A3
  #define PIN_FAULT     A6   // DRV8833 nFAULT，低 = 过流/过热
  #define PIN_VBAT      A7   // RAW_AA 经 22k/10k 分压

  #define HAS_MUX       1
  #define HAS_LATCH     1
  #define HAS_IR_SWITCH 1
  #define HAS_LEDS      1
  #define HAS_BUTTON    1
  #define HAS_VBAT      1
  #define HAS_FAULT     1

  #define NUM_LINE 5
  // HAS_MUX = 1 时填 74HC4051 通道号。巡线从左到右排列。
  static const uint8_t LINE_SRC[NUM_LINE] = {0, 1, 2, 3, 4};  // L2 L1 LC R1S R2S
  static const uint8_t OBS_SRC[2]         = {5, 6};           // OBSL OBSR
  #define MUX_CH_LATCH 7                                       // LATCHQ

#elif BOARD == BOARD_PERFBOARD
  #define PIN_L_IN1     6    // AIN1
  #define PIN_L_IN2     7    // AIN2
  #define PIN_R_IN1     9    // BIN1
  #define PIN_R_IN2     8    // BIN2
  #define PIN_ARM       10   // 直接接 SLP
  #define PIN_ENC_LA    2    // 洞洞板需另焊编码器线
  #define PIN_ENC_LB    3
  #define PIN_ENC_RA    4
  #define PIN_ENC_RB    5

  #define HAS_MUX       0
  #define HAS_LATCH     0
  #define HAS_IR_SWITCH 0
  #define HAS_LEDS      0
  #define HAS_BUTTON    0
  #define HAS_VBAT      0
  #define HAS_FAULT     0

  #define NUM_LINE 3
  // HAS_MUX = 0 时直接填模拟引脚
  static const uint8_t LINE_SRC[NUM_LINE] = {A0, A1, A2};     // 左 中 右
  static const uint8_t OBS_SRC[2]         = {A3, A6};         // 左 右避障（需另焊）
#else
  #error "未知 BOARD"
#endif

// ───────────────────────── 3. 传感器 ─────────────────────────
#define ADC_BITS            12
#define ADC_MAX             4095
#define MUX_SETTLE_US       20
#define IR_SETTLE_US        300    // TODO 发射管开关后光电管的稳定时间，示波器实测后可调小
#define SENSOR_ACTIVE_LOW   1      // 光电管上拉接法：反光越强，输出电压越低
#define LINE_IS_DARK        1      // 1 = 白底黑线；0 = 黑底白线
#define LINE_NOISE_FLOOR    0.15f  // 归一化后低于此值的“线强度”视为背景
#define LINE_FOUND_THRESH   0.45f  // 最暗的一路超过此值才认为看到线
#define CALIBRATE_MS        3000   // 校准时长：期间手推车体让每个探头扫过黑线和白底

#define OBS_MODE_ANALOG_DIFF 0     // 自制 IR 对管，发射管经 Q1 开关，取亮暗差值
#define OBS_MODE_DIGITAL_LOW 1     // FC-51 一类成品模块：有障碍输出低。模块必须用 3V3 供电
#ifndef OBS_MODE
#define OBS_MODE OBS_MODE_DIGITAL_LOW
#endif
#define OBS_THRESHOLD       600    // ANALOG_DIFF 模式的差值门限（12 位 ADC 计数）
#define OBS_DEBOUNCE        3      // 连续 N 帧结果一致才改变障碍状态

// ───────────────────────── 4. 电池 ─────────────────────────
#define VBAT_DIVIDER        3.2f   // (22k + 10k) / 10k
#define VBAT_ADC_REF        3.3f
#define VBAT_LOW_V          4.4f   // 4 × 1.10 V：LOW BAT 灯亮
#define VBAT_CRIT_V         4.0f   // 4 × 1.00 V：持续 VBAT_CRIT_MS 后停机
#define VBAT_CRIT_MS        500

// ───────────────────────── 5. 电机与编码器 ─────────────────────────
#define PWM_FREQ_HZ         20000  // 20 kHz，听不到电机啸叫
#define MOTOR_L_INVERT      0      // TODO 串口 't' 测试后，哪个轮子反转就把对应项改 1
#define MOTOR_R_INVERT      0
#define MOTOR_DEADBAND      0.02f

#define ENC_X4              0      // 0 = 只用 A 相边沿（×2，中断少一半）；1 = A/B 都用（×4）
#define ENC_L_INVERT        0      // TODO 轮子正转时计数应增加，否则改 1
#define ENC_R_INVERT        0
#define ENC_COUNTS_PER_REV  1400   // TODO 轮子转一圈的计数（电机 PPR × 减速比 × 2 或 4）
#define WHEEL_DIAMETER_MM   65.0f  // TODO
#define MAX_WHEEL_CPS       3000.0f // TODO 满 PWM、架空时的计数/秒，串口 't' 测试可读出

#define USE_SPEED_LOOP      0      // 编码器方向确认无误后改 1，启用轮速闭环
#define SPEED_KP            0.6f
#define SPEED_KI            4.0f

// ───────────────────────── 6. 运动控制 ─────────────────────────
#define CONTROL_HZ          100
#define AUTO_BASE_SPEED     0.45f  // 自主模式直道速度（-1..1）
#define LINE_KP             0.6f
#define LINE_KI             0.0f
#define LINE_KD             0.05f
#define LOST_SEARCH_SPEED   0.30f  // 丢线后朝最后一次看到线的方向原地转
#define LOST_TIMEOUT_MS     1500   // 超时仍找不到线就停机
#define MANUAL_MAX          0.8f   // 遥控模式最大输出

// ───────────────────────── 7. 通信与安全 ─────────────────────────
#define BLE_NAME            "MECH368-CAR-01"  // 每台车改成不同名字，遥控器按名字连接
#define LINK_TIMEOUT_MS     500    // 超过此时间没收到遥控器任何命令 → 急停
#define REQUIRE_LINK_IN_AUTO 1     // 1 = 自主模式也要求遥控器在线，遥控急停始终有效
#define TELEMETRY_MS        100
#define DEBUG_PRINT_MS      200
#define BTN_LONG_MS         1500
#define GESTURE_POLL_MS     50
