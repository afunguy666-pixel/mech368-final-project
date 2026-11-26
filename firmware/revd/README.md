# MECH 368 小车固件框架

| 目录 | 板子 | 作用 |
|---|---|---|
| `CarMain/` | 车上的 Nano 33 BLE Sense Rev2 | 巡线、避障、编码器、BLE 遥控与急停、安全联锁 |
| `Remote/` | 手持的另一块 BLE Arduino | 急停按钮、启动/恢复按钮、摇杆驾驶、心跳 |

## 准备
1. Arduino IDE → 开发板管理器：安装 **Arduino Mbed OS Nano Boards**，开发板选 *Arduino Nano 33 BLE*（Rev2 也选这个）。本机已装 4.6.0。
2. 库管理器：安装 **ArduinoBLE**（本机已装 2.1.1）和 **Arduino_APDS9960**（本机已装 1.0.4）。两个程序均已在 mbed_nano 4.6.0 上编译通过，无警告。
3. 打开 `CarMain/config.h`：
   - `BOARD`：`BOARD_REVD` 或 `BOARD_PERFBOARD`
   - `BLE_NAME`：每台车一个名字，并同步改 `Remote/Remote.ino` 的 `CAR_NAME`
   - `OBS_MODE`：成品避障模块（FC-51 一类，有障碍输出低）用 `OBS_MODE_DIGITAL_LOW`；自制 IR 对管用 `OBS_MODE_ANALOG_DIFF`
   - 标 `TODO` 的参数按实车测量后修改
4. `protocol.h` 在两个目录各有一份，改动时两份同时改。

## 文件
| 文件 | 内容 |
|---|---|
| `config.h` | 两种板的引脚表和全部可调参数 |
| `protocol.h` | BLE UUID、命令码、状态码、遥测包 |
| `CarMain.ino` | 状态机、100 Hz 控制循环、安全检查、串口命令、指示灯 |
| `motors.*` | 20 kHz PWM、ARM 锁存授权与回读 |
| `encoders.*` | 正交编码器中断计数 |
| `sensors.*` | 74HC4051 轮询、发射管亮/暗差分、电池、FAULT、锁存状态 |
| `line.*` | 探头校准、加权平均求线位置 |
| `link.*` | BLE 外设、命令队列、心跳超时 |
| `ui.*` | 状态灯、按键、板载 RGB |
| `pid.h` | PID |

## 状态机
```
            遥控 ARM_MANUAL / ARM_AUTO / 手势→ / 串口 m a
  DISARMED ───────────────────────────────────────────► MANUAL / AUTO
     ▲  ▲                                                   │
     │  └──────── 遥控 DISARM / 丢线超时 ────────────────────┤
     │                                                      │ 遥控 ESTOP、心跳超时 500 ms、板上按键、
     │ 遥控 CLEAR / 手势↑ / 串口 c                           │ 锁存被清零（欠压）、DRV8833 FAULT、电池过低
     └─────────────────────── ESTOP ◄───────────────────────┘
```
- 任何进入 ESTOP 或 DISARMED 的路径都会先把 ARM 拉低，SLEEP 立即变低。
- 从 ESTOP 只能回到 DISARMED，电机不会自己恢复；必须再发一次 ARM 命令，固件给出新的上升沿才能重新置位锁存器。
- RevD 欠压后锁存器被硬件清零，固件读到 LATCHQ = 0 会转入 ESTOP。
- `REQUIRE_LINK_IN_AUTO = 1` 时，自主模式下遥控器断开也会急停，保证遥控急停始终有效。

## 指示灯
| 灯 | 常亮 | 快闪 | 慢闪 |
|---|---|---|---|
| LINK | 遥控在线 | 已连接但心跳超时 / 因失联急停 | |
| ESTOP | 急停 | 因失联急停 | 正在校准 |
| LOW BAT | 低于 4.4 V | 低于 4.0 V | |
| 板载 RGB | 绿 = 遥控，蓝 = 自主，红 = 急停，黄 = 校准/测试，灭 = 待机 | | |

## 串口命令（115200）
`x` 急停 · `c` 解除 · `m` 遥控 · `a` 自主 · `d` 停车 · `k` 校准 · `t` 两轮测试 · `s` 开关实时打印 · `h` 帮助

## 首次调试顺序
**先把车架空，车轮离地。**
1. **不插驱动模块、不接电机**：上传，打开串口按 `s`，检查电池电压、各路探头读数（遮挡/放在黑线上时应变化）、`latch=0`。
2. **装驱动模块，接电池**：串口按 `t`。左轮先转 1 s，然后右轮转 1 s。
   - 车轮应向前转，否则改 `MOTOR_L_INVERT` / `MOTOR_R_INVERT`；
   - 打印的计数/秒应为正，否则改 `ENC_L_INVERT` / `ENC_R_INVERT`；
   - 用 30% 时的计数/秒估算 `MAX_WHEEL_CPS`（约为 3 倍）。
   - 如果提示 `latch did not set`，检查电池开关和 VSW（TP1）。
3. **巡线校准**：按 `k`，或长按板上按键 1.5 s。3 秒内手推车体，让每个探头都扫过黑线和白底。串口打印 `line cal OK` 才算成功。断电后需重新校准。
4. **遥控**：上传 `Remote`，配对后测试摇杆、急停、拔掉遥控器电源（车应在 0.5 s 内停）。
5. **自主**：先 `AUTO_BASE_SPEED` 取小值，从 `LINE_KP` 开始调，再加 `LINE_KD`。
6. 编码器方向全部正确后，把 `USE_SPEED_LOOP` 改为 1，再调 `SPEED_KP` / `SPEED_KI`。

## 已知限制和待定项
- 避障策略在 `CarMain.ino` 的 `obstacleStep()`，默认停车等待。按赛道规则改成绕行。
- RevD 所有 Nano 引脚都已占用。HC-SR04 这类 5 V 超声波模块无法经过 74HC4051；如果一定要用，只能占用 J7 的 A4/A5，并给 Echo 加分压到 3.3 V。
- 成品避障模块必须用 **3V3** 供电，否则输出高电平会超过 3.3 V。J6 第 1 脚是 VSW（5 V），不要用来给这类模块供电。
- `IR_SETTLE_US` 默认 300 µs 偏保守，用示波器看探头输出后可以调小。
- 编码器中断按 ×2 解码。电机转速很高时如果 CPU 占用过大，保持 `ENC_X4 = 0`。
