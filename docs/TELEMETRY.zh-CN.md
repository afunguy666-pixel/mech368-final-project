# MECH 368 遥测、预跑与在线轨迹对比

车端用现有编码器、里程计和陀螺仪估计轮轴中心位姿。手持板通过 USB 转发记录。电脑只接收数据，按键只标记日志。巡线、绕障、标定扫描和急停均在车端执行。第一版交付范围止于观测与对比，Assist 留到实车验收之后。

独立分支为 `feature/telemetry-compare`。`a3966b8` 保存开始时的 33 项本地改动，`b162d36` 保存工作期间原目录出现的 CAD 修订。本文数据采用后一个模型，D12、D11 两个避障探头均为水平朝前 0°。

## 本机运行

在独立 worktree 打开 PowerShell。以下命令可直接回放本次示例，或重新生成叠图。

```powershell
Set-Location 'D:\UBC\OneDrive - UBC\MECH 3\Mech368\mech368-telemetry'
.\scripts\telemetry.ps1 replay examples/telemetry/live.csv --speed 4
.\scripts\telemetry.ps1 compare examples/telemetry/teach.csv examples/telemetry/live.csv --plot build/replay-overlay.png
.\scripts\telemetry.ps1 verify examples/telemetry/live.csv examples/telemetry/live-ground.csv
```

在另一台 Windows 电脑上，使用以下命令安装依赖。启动脚本优先选用项目虚拟环境。本次工作目录也可直接使用已验证的本地 Python 3.12 和 `build/python-deps`。

```powershell
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools/requirements-telemetry.txt
```

用设备管理器确定**手持板**的 COM 号。以下示例使用 COM5。按 **S** 标出静止起点，过预先约定的地面分段线时按 **M**，两趟按相同顺序。按键作用于下一条收到的位姿。Windows 控制台和图窗都支持标记。关闭图窗或按 Ctrl+C 结束，CSV 文件名必须是新的。

```powershell
.\scripts\telemetry.ps1 record --port COM5 --run teach --output teach.csv
.\scripts\telemetry.ps1 record --port COM5 --run live --reference teach.csv --output live.csv
.\scripts\telemetry.ps1 compare teach.csv live.csv --plot overlay.png --matches matches.csv
```

实时窗显示 XY、当前日志段、赛道段和匹配状态。USB 重开或 BLE 重连都会另起 `log_segment`，先显示未对齐的车端坐标。将车放回固定起点、同一朝向且静止后再按 S。含多段的文件须以 `--teach-log N`、`--live-log N` 选段。`--headless` 关闭实时窗，`replay --summary` 只统计帧数。若只有事后同步的标记，用 `mark 原始.csv 标记.csv 新.csv`，标记表列为 `log_segment,robot_ms,course_segment`，每段从 0 连续递增。

## 线协议与安全边界

原 20 字节命令和安全状态继续使用 UUID 末段编号 0002、0003。独立遥测特征为 `c4130004-3680-4f01-9e10-627e70bca001`，只允许 Notify。采用默认 ATT 负载，所有多字节数都是**小端**，有符号数为二进制补码。坐标系为车启动时的轮轴中心，X 向前、Y 向左、逆时针为正。主机使用独立标出的起点重新表达坐标。[控制协议](BLE_PROTOCOL.zh-CN.md)

| 字节偏移 | 长度 | 内容 |
| --- | ---: | --- |
| 0 | 1 | 高四位版本 1，低四位类型 1 至 4 |
| 1 | 1 | 位 0 至 7 依次为导航运行、标定扫描、绕障、丢线、驱动使能、零偏就绪、急停锁存、数值越界 |
| 2 | 2 | 每类独立的 u16 采样序号，0 至 65535，按模回绕。拥塞跳过的时隙也占用序号 |
| 4 | 4 | 车端 u32 采样毫秒，约 49.7 天回绕。主机扩展为 `robot_ms`，USB 到达时间不参与里程对齐 |
| 8 | 10 | 下表的分类载荷，依次排列 |
| 18 | 2 | 前 18 字节的 CRC-16/CCITT-FALSE，初值 0xffff，多项式 0x1021 |

| 类型与频率 | 载荷、有效量程及缺测表示 |
| --- | --- |
| 1 位姿，10 Hz | i16 X/Y，每项 ±32767 mm。i16 航向为毫弧度，约 −3142 至 +3142。i32 轮轴中心有符号累计行程，单位 mm，倒车递减。i16 最小值、i32 最小值为无效，越界位禁止匹配。保护区从绝对值 2147483520 mm 起 |
| 2 四路 IR，1 Hz | 四个 u16 原始 ADC，各 0 至 1023，按 A0 车右外至 A3 车左外排列。i16 横向偏差 mm，左正，当前光斑几何约 ±30 mm。未跟到线或未在导航时为 −32768 |
| 3 编码器，1 Hz | 左、右各 i32 原始 decoded counts，按 32 位回绕，保留解码器原始正负号。末尾 i16 为静止 Z 轴 gyro bias，单位 0.001 °/s，±32.767 °/s，−32768 为无效 |
| 4 环境和状态，1 Hz | u16 A7 原始值 0 至 1023，u16 估算电池 mV。以 3300 mV 参考、暂定 3:1 分压时为 0 至 9900 mV。i16 温度单位 0.01 °C，传感器转换范围 −40 至 +125 °C，−32768 为无效。随后四个 u8 分别为导航状态 0 至 20、安全状态低四位、静止样本数 0 至 50、温度转换请求距采样的时间（每格 10 ms，255 表示无效） |

位姿和计数沿用车端 5 ms 更新拍的时间。导航中的 IR 来自同一控制拍，空闲时在诊断时隙实读。环境帧时间取 ADC 和传感器读取完成时刻。温度请求和读取分在两个时隙，通常相隔 300 ms。按温度年龄可定位转换请求，精度为 10 ms。HS3003 的 I²C 状态位和温度转换依据厂商数据手册。[Renesas HS3xxx 数据手册，第 7 节](https://www.renesas.com/en/document/dst/hs3xxx-datasheet?r=417406)

环境安全位从低到高依次为 ACK 有效、急停锁存、运动许可、电桥关闭。导航状态按 `NavState` 声明顺序，0 为空闲，1 为跟线，2 至 5 为弯道与稳定，6 至 7 为找线与终点确认，8 至 17 为绕行与回线，18 至 20 为终点、丢线停车和受阻。[状态枚举](../libraries/Mech368Line/src/LineNavigator.h)

USB 行以 `@` 开头，后接 60 个十六进制字符和换行。解码后的 30 字节依次为手持连接计数 u32、已验证控制状态中的车端会话号 u32、原始遥测 20 字节、覆盖前 28 字节的 CRC u16。接收器遇到半行可从下一个 `@` 恢复。每个持续连接内以 `log_segment,robot_ms` 关联诊断，每种帧的 `gap` 表示上一有效帧后缺失的数量。打开串口之前以及断开之后尚未观测到的帧数未知。掉线、重启后禁止跨段插值。

车端优先发布安全状态。遥测只在可用 HCI ACL 缓冲至少为 2 时尝试，给控制保留 1 个。满载时跳过样本，不补发。编译脚本在临时 ArduinoBLE 副本的 HCI 头文件加上只读的剩余缓冲查询，供单一 BLE 线程使用。普通应用通知在零缓冲时会等待。板上 USB 转发和车端调试均用 Mbed CDC 的 `send_nb`。磁盘写失败时主机报告一次并继续读取。手持 D2 常开对地按钮独立于 PC，原有单字符 `e` 急停仍可用串口终端触发。记录器独占 COM 时，使用 D2。

## 标定与独立对照

1. 在固定起点静置，车轮不转。收足 50 个静止 gyro 样本后固定零偏。转动编码器或检测到大角速度都会重新开始未完成的采样。手转每只轮的索引标记若干整圈，记录原始计数的带符号差 `ΔC` 与圈数 `N`。固件仅计 A 相的 CHANGE 边沿。每轮 `sign = sign(ΔC/N)`，`CPR = |ΔC/N|`。
2. 按比赛负载在有直尺的地面直推，得独立距离 `D` 和每轮 `ΔC`。每轮滚距为 `|D| × CPR / |ΔC|`。分别做顺、逆时针原地转动，以外部量角器或视频测量 `Δθ`（输入为度，计算转为弧度）。有效轮距为 `(Δs右 − Δs左) / Δθ`。重复同类测量，工具取中位数。跨编码器回绕时按 32 位模差计算原始增量。[标定输入示例](../examples/telemetry/bench-trials.csv)
3. 用 `.\scripts\telemetry.ps1 calibrate trials.csv --header build/MeasuredDrive.h` 计算结果。实测值可填入 `DriveCalibration.h` 的左右轮与轮距部分。仓库默认值仍为直径 65 mm、轮距 92 mm 和每轮 600 decoded counts/rev。车端复用 `Odometry`、`GyroBias` 和既有速度 PI，同一组轮尺用于速度和位姿。
4. 同一标记放置轴心和车头，两趟同向，在独立地面或视频门线上记录分段。工具仅使用固定起点旋转和平移，按**同段、段内有符号里程**内插 teach。参考帧须连续，间距至多 85 mm、时间至多 250 ms。航向差超过 30°、位置差超过 100 mm，或另一平行段（含反向）进入候选误差加 25 mm 的范围时，拒绝匹配。掉包、倒退、标定、急停、绕障、丢线和不确定的分段标记均影响置信度。这些是第一版观察阈值。
5. 把未参与标定与分段的实地检查点整理为 `log_segment,robot_ms,x_m,y_m`，原点与朝向同固定起点。俯视视频应标尺定标，以共同可见的事件同步到车端时间。用 `verify` 对两组参数分别计算同组检查点的 XY RMS 和最大误差，记录覆盖点数、地面测量与时间同步误差。教跑与在线轨迹互相接近只说明重复性。

## 本次仿真结果

两趟均使用仓库的 `course.world`、`ir_tracker.vehicle` 和实际 robot/remote sketch。记录来自手持 USB 字节流。独立对照来自物理模型输出的车轴中心，沿四条车道各保留两个独立检查点。示例坐标原点为固定起点，S1 至 S3 取于三个横向连接段。

| 项目 | 结果 |
| --- | --- |
| teach / live 两趟 | 随机种子 368 / 369，分别于 49.603 s、49.643 s 发出终点停车指令，均绕过三个锥桶，模型碰撞数 0。teach 的稳态位姿为 10.0 Hz |
| USB 拥塞注入 | live 缺 6 个位姿帧、1 个编码器帧和 1 个环境帧。叠图在缺口断线 |
| 同段同里程对比 | 144 个可靠对应点，XY RMS 9.0 mm。两个平行段歧义样本被排除。绕障和丢线可同时出现在同一帧 |
| 被动负载台架 | 左、右前向符号均为 +1，分别为 608、596 counts/rev。滚距为 195.002、199.138 mm/rev，有效轮距 94.961 mm。负载赛道静止偏置约 +0.649 °/s |
| 独立负载赛道 | 同一带轮尺偏差的物理模型、同一随机种子，分别运行 CAD 与台架参数。8/8 留出点的 XY RMS 从 39.8 降至 1.5 mm，最大误差从 52.0 降至 2.1 mm。标定仅应用于另一份仿真构建 |
| 主机与固件 | 5 个 C++ 测试程序、7 个 Python 回归测试和 33 个固件场景通过。原有 SIL 生成 5 组输出。Windows 原生 Tk 图窗渲染通过，pySerial 软件回环录入了来自模拟手持板的 35 帧 |
| 故障与急停 | 5 个安全场景各跑种子 1 至 10，50/50 通过。三种断流工况 30 次实测虚拟 D0 有效包边沿至 D8 拉低，最慢 451.003 ms。D2 急停在 USB 拥塞时有效，急停后重连保持桥关闭 |
| 实际 sketch 编译 | Nano 目标 `arduino:mbed_nano:nano33ble`，核心 4.6.0，ArduinoBLE 2.1.1。默认 robot 占 378728 字节 Flash，remote 占 327848 字节 |

目前没有实车轮尺、COM 实物链路记录或俯视视频。模型的轮地接触、无线排队、传感器和 USB 时序是近似值。表内误差与停驱动时间用于本次软件验证。实车数据请填入[一页双板验收记录](../output/pdf/two-board-bench.zh-CN.pdf)。

[teach CSV](../examples/telemetry/teach.csv) · [live CSV](../examples/telemetry/live.csv) · [轨迹叠图](../examples/telemetry/overlay.png) · [逐帧匹配结果](../examples/telemetry/matches.csv) · [负载标定逐点对照](../examples/telemetry/calibration-check.csv) · [停驱动时序](../examples/telemetry/safety-timing.csv)

## 重建

Windows 上实际双板编译可使用 Arduino IDE 附带的 CLI。以下命令只编译，输出在 `build/arduino-robot` 和 `build/arduino-remote`。台架参数 `--trace-accepted` 另生成带 `-bench` 的两个目录，并启用车端 D0 有效包测点和手持 D3 接地中断心跳。D2 急停在两种构建中都优先。

```powershell
$python = "$env:USERPROFILE\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
$cli = "$env:USERPROFILE\AppData\Local\Programs\arduino-ide\resources\app\lib\backend\resources\arduino-cli.exe"
& $python scripts/compile_firmware.py --cli $cli
& $python scripts/compile_firmware.py --cli $cli --trace-accepted
```

本机已有的 Git Bash 和主机编译器也可直接从 PowerShell 调用。沿用上段的 `$python`。

```powershell
$bash = 'C:\Program Files\Git\bin\bash.exe'
$env:CXX = "$env:USERPROFILE/AppData/Local/Temp/mech368-w64devkit-2.11.0/w64devkit/bin/g++.exe"
$env:PYTHON = $python
& $bash scripts/test-host.sh
& $bash scripts/run-sil.sh
& $bash scripts/run-fw-sim.sh
```

以下在装有 Python 3 与 C++17 编译器的 Bash 中执行。示例生成命令使用新目录和新文件名。

```bash
bash scripts/test-host.sh
bash scripts/run-sil.sh
bash scripts/run-fw-sim.sh
bash scripts/run-fw-sim.sh --seeds 10 robot_track_watchdog telemetry_deadline telemetry_usb_power_loss telemetry_pressure robot_track_estop
build/fwsim-build/fwsim --quiet --seed 369 --out build/fwsim telemetry_live_course
python3 tools/sim_measurements.py lap build/fwsim/robot_track_cones teach build/example
python3 tools/sim_measurements.py lap build/fwsim/telemetry_live_course-seed369 live build/example
python3 tools/telemetry.py compare build/example/teach.csv build/example/live.csv --plot build/example/overlay.png
python3 tools/sim_measurements.py bench build/fwsim/telemetry_bench build/trials.csv
python3 tools/telemetry.py calibrate build/trials.csv --header build/FittedDrive.h
FWSIM_DRIVE_PROFILE=build/FittedDrive.h bash scripts/run-fw-sim.sh -q --out build/fitted telemetry_loaded_course
python3 tools/sim_measurements.py lap build/fwsim/telemetry_loaded_course live build/cad-check
python3 tools/sim_measurements.py lap build/fitted/telemetry_loaded_course live build/fitted-check
python3 tools/telemetry.py verify build/cad-check/live.csv build/cad-check/live-ground.csv
python3 tools/telemetry.py verify build/fitted-check/live.csv build/fitted-check/live-ground.csv
```

## 变更清单

| 入口 | 变更 |
| --- | --- |
| `firmware/robot`、`Mech368Tracker` | 采样位姿与诊断、左右轮独立标尺、静止零偏锁定、可丢弃的调试输出。标定和巡线仍经原有 `MotionSafety` 闸门写电桥 |
| `firmware/remote`、`Mech368BLE` | 独立 Notify 编解码、序号和时间、USB 转发、D2 手持急停及台架测点 |
| `tools`、`scripts/telemetry.ps1` | Windows COM 记录、实时显示、回放、校准计算、赛道对齐、误差与独立对照 |
| `sim/fwsim`、`tests`、构建脚本 | 虚拟台架、USB 和 HCI 拥塞、供电中断、有效包到电桥的时序测试，以及实际 Nano 双板构建 |
