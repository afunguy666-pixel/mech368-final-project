# 固件在环（firmware-in-the-loop）模拟器

[English](SIMULATOR.md) / 中文版

> 本文是 [SIMULATOR.md](SIMULATOR.md) 的中文对照版。代码、注释和程序输出都是英文；两个版本如有出入，以英文版为准。

在小车造出来之前，就能在电脑上测试**未经修改的**机器人和遥控器程序。
`scripts/run-fw-sim.sh` 会把 `firmware/robot/robot.ino`、`firmware/remote/remote.ino`
和示例程序，连同一组在电脑上运行的替身库一起编译。替身库覆盖 Arduino 核心库、ArduinoBLE、
Rev2 的 IMU（`Arduino_BMI270_BMM150`）、手势传感器（`Arduino_APDS9960`）和温度传感器
（`Arduino_HS300x`）。然后两块虚拟开发板在虚拟时钟上运行，通过模拟的 BLE 无线链路相连。
机器人板连着一辆虚拟差速小车，小车跑在虚拟场地上。

它和 `sim/Simulation.h` 里的运动学 SIL（`scripts/run-sil.sh`）是两个不同的工具。
那个是围绕可移植的 `Receiver` 重新实现一遍主循环；这个运行的是你真正的
`setup()`/`loop()`/BLE 回调。

```bash
bash scripts/run-fw-sim.sh
```

上面这条运行全部场景，并把回放写到 `build/fwsim/`。其他命令：

```bash
bash scripts/run-fw-sim.sh estop_latch
```

```bash
bash scripts/run-fw-sim.sh --seeds 20 link_stability
```

```bash
bash scripts/run-fw-sim.sh --list
```

第一条运行单个场景，并输出完整的事件和串口日志。第二条用种子 1..20 运行一个场景
（对无线时序和噪声做蒙特卡洛测试）。第三条列出所有场景。
需要 Bash 和 C++17 编译器（用 `CXX` 替换默认的 `c++`）。
不需要 Arduino CLI、开发板、Python 或网络。

每次运行会得到：

| 文件 | 内容 |
| --- | --- |
| `<场景>.html` | 独立的回放页面：俯视图、机器人轨迹、传感器射线、LED、链路/状态标志、逻辑分析仪条、事件和串口日志、检查结果。任何浏览器都能打开。 |
| `<场景>.csv` | 每 10 ms 一行：位姿、轮速、电机占空比、电池、偏离线距离、LED、链路、状态标志、传感器读数 |
| `<场景>.map.csv` | 程序打印的温度地图点（`MAP x y 温度`），附带机器人的真实位置和该处的真实温度 |
| `<场景>.frames.csv` | 空中的每一个 BLE 报文：发送/到达时间、方向、结果（送达/丢失/阻断/损坏/链路断开）、解码后的 Mech368 字段 |
| `<场景>.log` | 带时间戳的事件、两块板的 `Serial` 输出以及检查结果 |

## 模拟了哪些东西

| 部分 | 模型 |
| --- | --- |
| 时间 | 每块板有自己的虚拟微秒时钟。每次 `loop()` 耗时 1 ms，外加 0–0.3 ms 随机抖动。API 调用耗时几微秒，所以基于 `millis()` 的忙等循环会正常结束。`delay()` 会推进时钟。单次 `setup()`/`loop()` 调用卡住超过 20 s 虚拟时间，就会中止运行。 |
| 引脚 | `pinMode`、`digitalWrite/Read`、`analogWrite/Read`（含分辨率设置）、`attachInterrupt`（CHANGE/RISING/FALLING）、`noInterrupts`、`pulseIn`，以及 Nano 33 BLE 的引脚编号（`D0`–`D13`、`A0`–`A7`、`LEDR/G/B`）。 |
| 串口 | 输出会带时间戳记录下来。场景脚本可以向串口输入字符。 |
| BLE | ArduinoBLE 1.4 外设端和中心端的常用子集：广播/扫描/连接/发现服务/订阅、无响应写入、通知、事件回调（在 `BLE.poll()` 内执行）。无线链路：10 ms 加 0–5 ms 延迟，按顺序送达，2 s 监督超时，连接/发现/订阅这些阻塞调用的耗时是随机的。可注入的故障：丢包、比特翻转、单向阻断（"连着但不通"）、超出范围、开发板断电。 |
| 小车 | 电机驱动类型 `pwm_dir`、`in_in`（DRV8833 式）、`en_in_in`（L298N/TB6612 式），都可以加一个可选的 `sleep=` 引脚，它为低电平时驱动输出关闭。电机转速一阶响应，随电池电压缩放，带死区；精确的差速运动学；会触发你的中断函数的正交编码器；碰撞检测用一个圆（半径 `body_radius`，圆心在轮轴前方 `body_offset` 处）。 |
| 传感器 | 胶带上方的模拟/数字反射式巡线传感器；HC-SR04 式超声波（触发脉冲 + `pulseIn`）；Sharp 式模拟红外测距；经比较器输出的反射式红外避障对管（`ir_proximity`：物体进入 `fov_deg` 波束内的 `range` 距离时数字输出翻转）；比较器输出的巡线传感器也能用 `analogRead` 读（0 V / 3.3 V）；电池分压；由小车运动算出的 IMU 陀螺仪/加速度计/磁力计（带零偏和噪声）；由脚本给出的 APDS 手势和接近度；HS300x 温度取自场地温度场，带一阶滞后、噪声和偏移（假定每次读取阻塞约 10 ms）。遥控器的 IMU 跟随脚本设定的手部倾斜。 |
| 场地 | `sim/worlds/*.world`：胶带线、椭圆环、方块、圆形障碍、直角或圆角边界墙，以及温度场（`ambient` 加上 `heat` 热点/冷点）。固件链路场景都在 `course.world` 上运行。 |

车辆文件描述硬件，场地文件描述赛道。

- `sim/vehicles/ir_tracker.vehicle` 和 `ir_tracker_lm393.vehicle` 采用原生整车装配的几何。`nose_down_deg` 设置车头向下的静态角度，用于 IMU 和巡线光轴与地面的交点。避障探头的位置直接用地面坐标表示。可选的 `body_plan PNG xmin ymin xmax ymax` 读取车辆文件旁边的图片并嵌入回放。边界坐标以米为单位，相对轮轴中点，x 向前、y 向左。[车体几何](LINE_FOLLOWING.zh-CN.md#车体几何)
- `sim/vehicles/example.vehicle`：引脚、电机驱动类型、轮径和轮距、编码器 PPR、传感器位置、电池分压比、陀螺仪零偏。
- `sim/worlds/course.world`：按课程讲义图 6 描出的期末项目场地（5 cm 黑色赛道、蛇形路线、三个锥桶、圆角场地边界）。
  图上没有尺寸，所以布局按假设的约 2.0 m × 1.5 m 比例绘制。请去 KAIS 1220 量一下真实赛道，再更新坐标。
  热区（+12 °C）、冷区（−8 °C）和 22 °C 的环境温度也都是假设值。
  `oval.world` 和 `oval_obstacle.world` 是简单的练习用环形赛道。

## 编写测试场景

场景是 `sim/fwsim/scenarios/` 里的 C++ 文件。每个场景在一个新的进程中运行，
所以程序的全局变量每次都从上电状态开始。

```cpp
#include "Simulation.h"
using namespace fwsim;

FWSIM_SCENARIO(my_estop_test, "E-stop during a link loss still latches") {
  sim.loadWorld("sim/worlds/course.world");
  sim.loadVehicle("sim/vehicles/example.vehicle");
  sim.useSketch(Robot, "robot");            // firmware/robot/robot.ino
  sim.useSketch(Remote, "remote", 300);     // 300 ms 时上电
  sim.linkLoss(4000, 5000);                 // 4–5 s 超出范围
  sim.serialInput(Remote, 4500, "e");       // 操作员在此期间按急停
  sim.runUntil(9000);
  FWSIM_CHECK(sim, sim.blinking(Robot, LED_BUILTIN, 6000, 9000), "robot LED flashes");
}
```

| 输入激励 | 作用 |
| --- | --- |
| `serialInput(board, ms, "text")` | 向某块板的 `Serial` 输入文字 |
| `gesture(ms, GESTURE_UP)` | 在机器人前方做一个 APDS 手势 |
| `tilt(ms, rollDeg, pitchDeg)` | 遥控器手部姿态的关键帧（中间自动插值） |
| `setInput(board, ms, pin, level)` / `setAnalog(board, ms, pin, volts)` | 强制设定按钮或模拟输入 |
| `setBattery(ms, volts)` | 电池电压突变 |
| `linkLoss(fromMs, toMs)`、`block(remoteToRobot, fromMs, toMs)`、`powerOff(board, ms)` | 无线和电源故障 |
| `at(ms, [](Simulation& s) { ... }, "标签")` | 其他任何操作，例如 `s.radio().lossCentralToPeripheral = 0.1` 或 `s.world().boxes.clear()` |
| `seed(n)`、`board(b).timeOffsetUs` | 随机种子；让 `millis()` 从接近 32 位溢出的值开始 |

| 观测 | 作用 |
| --- | --- |
| `blinking`、`steady`、`levelAt`、`transitions` | 任意一块板上的任意输出引脚（LED 等） |
| `frames()`、`statusSentBefore(ms)`、`lastCommandDelivered(ms)` | 解码后的 BLE 通信 |
| `vehicle()`（`pose`、`travelled`、`collisions`、`distanceReading` 等）、`maxLineOffset` | 物理状态 |
| `serialContains(board, "text")`、`hasEvent("text")`、`samples()` | 日志和轨迹 |
| `mapPoints()`、`scoreMap(cellSize)` | 温度地图点及评分（见下文） |

`sim.levelAt(...)` 里的引脚就是程序中使用的同一批常量（`LED_BUILTIN`、`LEDR` 等）。
Nano 33 BLE 的 RGB 引脚上的 LED 是低电平点亮。

## 温度建图

任一块板在 `Serial` 上打印的 `MAP x y 温度`（或 `MAP,x,y,温度`）格式的行，都会被收集为地图点。
模拟器记录打印那一刻机器人的真实位置。如果数据是经遥控器转发后才打印的，
打印时间会更晚，这部分延迟会计入误差。随后 `scoreMap()` 和回放页面的 "Temperature map" 面板
会给出讲义中的建图指标：

| 指标 | 含义 |
| --- | --- |
| 点数、唯一点数、密度 | 记录的点数；位置不同（精确到 1 cm）的点数；每平方米场地的唯一点数 |
| 覆盖率 | 含有数据点的 0.2 m 网格单元所占比例 |
| 位置误差 | 记录的位置与打印时真实位置之差（平均值、最大值） |
| 温度误差 | 记录的温度与机器人实际所在位置真实气温之差 |
| 地图误差 | 记录的温度与"记录位置"处真实温度之差（即热力图所呈现的误差） |

回放页面底层画出真实温度场，上层画出记录的点，两者使用同一色标。
运行 `course_heat_map` 可以看到示例程序的效果。示例程序在 `Serial` 上收到
`pose X Y 航向角(度)`（已知的起始位姿）后开启建图；等待期间校准陀螺仪零偏；
用编码器测距离、用陀螺仪测航向来估计位姿。

## 测试你自己的程序

每个 `firmware/<名字>/<名字>.ino` 和 `sim/examples/<名字>/<名字>.ino`
都会以程序名 `<名字>` 自动编译进来。想试一个改动又不想动原文件，
可以把程序文件夹复制到别处，给文件夹和 `.ino` 改名，然后替换进现有场景：

```bash
FWSIM_SKETCHES=/path/to/remote_v2 bash scripts/run-fw-sim.sh --seeds 10 --sketch remote=remote_v2
```

`sim/examples/line_follower` 是演示程序（不是课程固件）。它用编码器做电机速度闭环、
巡线、遇障停车、用手势启停、低电量时点亮红色 LED，并演示 vehicle 文件里的引脚
如何对应到普通的 Arduino 代码。

`sim/examples/line_tracker` 在 `sim/vehicles/ir_tracker.vehicle` 和
`ir_tracker_lm393.vehicle` 上运行 `libraries/Mech368Line`（标定后的红外阵列、PD 巡线、
90° 弯道、绕锥桶）。它的 `track_*` 场景见 [LINE_FOLLOWING.zh-CN.md](LINE_FOLLOWING.zh-CN.md)。
`robot_track_*` 场景在直连车型上运行车载和遥控两份固件，检查整圈绕障、心跳中断时的 D8 电平、标定中断后的重启，以及急停和本地向上手势同时到达时的优先级。

## 对第一阶段固件的发现（2026-11-21）

- **看门狗、急停锁存、坏包和断电处理在所有随机种子下都符合规格**：
  每条报告"允许运动"的状态，都在一条已送达的运动指令之后 450 ms 以内；
  任何损坏的报文都没有延长看门狗；急停锁存在断链和重连后仍然保持。
- **已修复：遥控器会间歇性地断开一条正常的链路。** 遥控器原来用自己独立的
  50 ms 定时器发指令，与机器人的 50 ms 状态定时器互不同步。两者相位对不上时，
  每条指令带的是"上上一条"状态里的挑战码，于是机器人把每隔一条指令当作过期拒收，
  ACK 也永远对不上遥控器最近发出的序列号。遥控器随后判定链路不健康并重连
  （约 1.5–2 s 无法控制）。`link_stability` 在 10 个种子下全部失败；
  还有一次急停在请求后 815 ms 才锁存，因为请求正好落在一次这样的重连期间。
  现在 `firmware/remote/remote.ino` 在每收到一条有效状态通知后发送恰好一条指令，
  与 `docs/BLE_PROTOCOL.md` 的描述一致。修复后，当时的全部 14 个场景在 10 个种子下都通过了。
  这个修复只在模拟中验证过：请在两块真实开发板上确认链路稳定性和急停延迟。
  需要注意的副作用：如果状态通知停了，遥控器也会停止发送（包括急停报文）。
  不过那种情况下它的指令本来就会带着过期的挑战码，而机器人的看门狗会让运动停止。
- **示例程序在建图上的教训**（模拟所得，同样的机理在开发板上也存在）：
  - 把每个陀螺仪读数按固定 10 ms 积分（`gz * 0.01`），在六个 90° 弯里一共少算了约 8°。
    阻塞的温度读取和 `pulseIn` 调用会让主循环跳过 IMU 样本。记录的位置因此漂移到
    平均 17 cm、最大 42 cm 的误差。改为按两次读数之间实测的时间（`micros()`）积分后，
    误差降到 1.6 cm / 6.4 cm。
  - 传感器时间常数 2 s、车速 0.25 m/s 时，每个读数描述的是沿路径约 0.5 m 之前的空气温度，
    热力图会被"拖尾"（平均误差约 1.6 °C）。请实测真实 HS300x 的响应时间，
    然后在热区和冷区附近开慢一点，或者把读数标记到更早的位姿上。
- **场地几何（按假设比例）：场地边界离赛道很近。** 在 `course.world` 中，
  赛道每个底部横段距离边界约 19 cm，每个顶部横段约 24 cm。
  像示例程序那样"前方 20 cm 内有东西就停车"的简单规则，如果边界是实体墙，
  会在转弯之前就对着边界停下。请确认真实场地的情况。障碍检测或许应该只考虑赛道上
  或赛道附近的物体，或者在接近竖段尽头时使用更小的阈值。

## 局限：模拟通过并**不能**说明什么

- ArduinoBLE、HCI 和无线链路都是带假设时序的行为模型，不是真实的协议栈。
  真实的延迟、阻塞时间、流控和监督超时仍需在两块开发板上测量。
- Arduino API 的替身是一个宽松的子集，例如 `digitalWrite(pin, 1)` 在这里能编译。
  能否编译以开发板构建（`scripts/compile-firmware.sh`）为准。
  电脑上的构建**不会**像 Arduino 那样自动生成函数原型：函数要先声明再使用。
  程序文件夹里自带的 `.cpp` 文件不会被编译。
- 除了 ArduinoBLE、`Arduino_BMI270_BMM150`、`Arduino_APDS9960` 和 `Arduino_HS300x`
  （以及本仓库的 `libraries/`）之外，其他库都不可用。如果需要，请在
  `sim/fwsim/hostduino/` 下添加替身。
- 温度场是平滑的假设模型，没有气流，也不考虑机器人自身对传感器的加热。
- 每次运行只上电一次：开发板可以断电，但在同一场景内不能复位或重新上电。
- 小车物理模型没有车轮打滑、底盘惯性、电机饱和电流或带载时的电池压降。
  碰撞只阻挡平移。IMU 坐标轴假定开发板水平安装，x 向前，z 向上。
  中断瞬间执行完毕，最多只嵌套一层。
- `delay()` 期间不处理 BLE 事件，只有 `BLE.poll()` 和阻塞的 BLE 调用会处理，与真实库一致。

模拟结果只是"代码逻辑在所建模条件下"的证据，永远不能代替
[BLE_PROTOCOL.zh-CN.md](BLE_PROTOCOL.zh-CN.md) 和 README 中的台架检查。
