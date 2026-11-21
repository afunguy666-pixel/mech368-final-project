# 第一阶段评审 — 等待批准

基线：`c8ab0aba27543d3209b08c5290a7b848492d6815`（克隆时的 main 最新提交）。
给定的 `6fe2986` 是它的祖先提交；之后添加的初始化/测试注释予以保留。
尚未进行任何提交、推送、分支发布或开发板烧录。

拟定的提交标题：`Add phase-one BLE protocol, safety integration and host tests`

## 准确的改动范围

| 文件 | 改动 |
| --- | --- |
| firmware/robot/robot.ino | BLE 外设、逐次写入校验、状态通知、软件闸门和板载 LED |
| firmware/remote/remote.ino | 中心端的扫描/连接/发现/订阅/重试状态机、零速心跳、ACK 超时、"粘住"的串口急停、板载 LED |
| libraries/Mech368BLE/library.properties | 新的可移植库的元数据 |
| libraries/Mech368BLE/src/Mech368Protocol.h | 20 字节编解码、CRC、字段范围、连接/挑战码/序列号新鲜度、接收器适配层和遥控器 ACK 跟踪 |
| tests/ble_protocol_test.cpp | 独立的协议、安全集成以及模拟交换/丢失测试 |
| scripts/test-host.sh | 以 C++11 编译并运行所有主机测试源文件，警告视为错误 |
| .github/workflows/ci.yml | 增加固定版本 ArduinoBLE 1.4.1 依赖 |
| README.md | 描述已实现的阶段和剩余工作 |
| docs/BLE_PROTOCOL.md | 精确的协议、局限和尚未执行的台架检查清单 |
| docs/PHASE1_REVIEW.md | 本评审及验证记录 |

原有的 MotionSafety 头文件、它的测试、编译脚本和目录结构都予以保留。
没有添加任何电机引脚、PWM 调用或电机驱动器写入。

## 2026-11-20 的本地验证

- `bash scripts/test-host.sh`：motion_safety_test.cpp 和 ble_protocol_test.cpp 均通过，
  包括带"冻结重放/丢失"的模拟交换。
- AddressSanitizer + UndefinedBehaviorSanitizer 下的协议测试：在关闭泄漏检测的情况下通过。
  本环境的 ptrace 插桩不支持 LeakSanitizer，因此不声称有泄漏检查结果。
- `bash scripts/compile-firmware.sh`：在 Arduino CLI 1.3.1、arduino:mbed_nano 4.4.1、
  ArduinoBLE 1.4.1、目标 arduino:mbed_nano:nano33ble 下通过。
- 机器人：327256 字节 flash（33%）；69712 字节全局变量（26%）。
- 遥控器：327648 字节 flash（33%）；69784 字节全局变量（26%）。
- 工具链压缩包通过可用的下载途径获取，并用 Arduino 软件包索引中的 SHA 校验和核对；
  本环境无法使用 CLI 的正常网络下载。
- `git diff --check`：通过。这次尚未提交的改动没有 GitHub Actions 运行记录。

## 重要局限

没有双板硬件测试、时序测量、物理输出验证或电气安全验证。遥控器只发送零速运动。
APDS 恢复、IMU 转向和实体急停输入仍未实现。串口 `e` 只是台架触发方式。
没有 GPIO 电机输出，并不能证明电气上已经禁止。

应用层调度是协作式的；ArduinoBLE 的连接/发现和 HCI 操作仍可能阻塞。
100 ms 的 ATT 超时并不能限制整个服务发现的耗时。450 ms 的软件闸门经过了测试，
但 ≤500 ms 的物理停车和不间断的 LED 响应尚未得到证实。
会话号/挑战码只是新鲜度检查，不是经过认证的配对，也不是持久的防重放保护。

只有在审阅补丁之后才请求批准：从此基线创建一个单独的功能分支，
提交并推送这十个文件，保持 main 不变。发布前重新检查远端最新提交；
如果它已经变化，先保留并整合上游改动再继续。
