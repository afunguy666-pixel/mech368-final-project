// 虚拟场地（赛道线、障碍物）以及接在模拟开发板上的虚拟硬件：
// 机器人板连着一辆差速小车，遥控器板连着一只脚本控制的手（倾斜）。
// 这里的每个数字都是假设值，
// 需要换成台架实测数据。
#ifndef FWSIM_MODEL_H
#define FWSIM_MODEL_H

#include "Runtime.h"
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace fwsim {

struct Vec2 { double x = 0, y = 0; };
struct Pose2 { double x = 0, y = 0, yaw = 0; };  // 米，弧度（逆时针为正）

// ----------------------------------------------------------------- 场地 ---
struct World {
  struct Line { std::vector<Vec2> pts; double width = 0.019; };
  struct Box { Vec2 lo, hi; std::string name; };
  struct Circle { Vec2 c; double r = 0; std::string name; };
  struct Wall { Vec2 a, b; std::string name; };
  // 温度凸起：ambient + delta * exp(-(d / radius)^2)。
  struct Heat { Vec2 c; double radius = 0, delta = 0; std::string name; };

  double width = 3.0, height = 2.0;
  Pose2 start;
  std::vector<Line> lines;
  std::vector<Box> boxes;
  std::vector<Circle> circles;
  std::vector<Wall> walls;
  double ambient = 22.0;  // 摄氏度
  std::vector<Heat> heat;
  std::string source = "（内置的空白场地）";

  static World load(const std::string& path);
  // p 到最近赛道中心线的距离（没有赛道时为 inf）。
  double lineCentreDistance(Vec2 p) const;
  // 给定半径的传感器光斑被赛道覆盖的比例（0..1）。
  double lineCoverage(Vec2 p, double spotRadius) const;
  // 沿射线到第一个障碍物的距离，超过 maxRange 时为 +inf。
  double raycast(Vec2 origin, double yaw, double maxRange) const;
  bool collides(Vec2 centre, double radius, std::string* what) const;
  double lineLength() const;
  double temperature(Vec2 p) const;  // 真实气温，摄氏度
};

// --------------------------------------------------------------- 小车 ---
enum class Driver { None, PwmDir, InIn, EnInIn };
struct MotorConfig {
  Driver driver = Driver::None;
  int pwm = -1, dir = -1, in1 = -1, in2 = -1, en = -1;
  bool invert = false;
};
struct EncoderConfig { int a = -1, b = -1; double ppr = 0; bool invert = false; };
struct LineSensorConfig {
  std::string name;
  int pin = -1;
  bool analog = true;
  double x = 0, y = 0;                  // 底盘坐标系：x 向前，y 向左（米）
  double onVolts = 2.8, offVolts = 0.4; // 在赛道上 / 在地面上时的模拟输出
  int onLevel = 1;                      // 在赛道上时的数字输出电平
  double spot = 0.004;                  // 感应光斑半径（米）
};
struct DistanceConfig {
  enum Kind { Ultrasonic, AnalogIR } kind = Ultrasonic;
  std::string name;
  int trig = -1, echo = -1, pin = -1;
  double x = 0, y = 0, yaw = 0;  // 安装位姿（米，弧度）
  double minRange = 0.02, maxRange = 2.0, noise = 0.002;
};
struct BatteryConfig {
  int pin = -1;
  double divider = 3.0;  // V_电池 / V_引脚
  double volts = 7.4, drainPerMinute = 0.0;
};
struct VehicleConfig {
  double trackWidth = 0.10, wheelRadius = 0.020, bodyRadius = 0.07;
  double maxWheelSpeed = 30.0;    // rad/s，空载、满占空比、额定电压
  double timeConstant = 0.08;     // 秒，电机一阶响应
  double deadband = 0.06;         // |占空比| 低于此值时车轮不转
  double nominalVolts = 7.4;
  double leftScale = 1.0, rightScale = 1.0;  // 有效车轮半径系数
  double gyroBiasDps = 0.0, gyroNoiseDps = 0.0;
  double tempTau = 2.0, tempNoise = 0.05, tempOffset = 0.0;  // HS300x 模型
  MotorConfig left, right;
  EncoderConfig encLeft, encRight;
  std::vector<LineSensorConfig> lineSensors;
  std::vector<DistanceConfig> distance;
  BatteryConfig battery;
  std::string source = "（内置的默认小车）";

  static VehicleConfig load(const std::string& path);
};
int parsePin(const std::string& text);

// 任何开发板都可以有的脚本输入（手势、接近度、手部倾斜）。
class ScriptedIO : public BoardIO {
 public:
  std::vector<std::pair<uint64_t, int>> gestures;          // 时间，GESTURE_*
  std::vector<std::pair<uint64_t, Vec2>> tilt;             // 时间，（横滚，俯仰）度
  int proximityValue = 255;

  bool gestureDue(Board& b) override;
  int takeGesture(Board& b) override;
  int proximity(Board&) override { return proximityValue; }
  ImuSample imu(Board& b) override;  // 手持开发板：只有倾斜
  Vec2 tiltAt(uint64_t t) const;
};

class Vehicle : public ScriptedIO {
 public:
  Vehicle(const VehicleConfig& config, const World& world);

  VehicleConfig cfg;
  const World& world;
  Board* board = nullptr;

  Pose2 pose;
  double omegaL = 0, omegaR = 0;   // 车轮 rad/s
  double thetaL = 0, thetaR = 0;   // 车轮转角，弧度
  double dutyL = 0, dutyR = 0;     // 带符号的电机指令 -1..1
  double speed = 0, yawRate = 0, accel = 0;
  double batteryVolts = 7.4;
  double sensedTemp = 0;  // 传感器处经过滞后的温度，摄氏度
  double travelled = 0;
  uint64_t timeUs = 0, stepUs = 500;
  bool colliding = false;
  int collisions = 0;

  std::function<void(uint64_t, const std::string&)> onEvent;
  std::function<void(uint64_t)> onSample;
  uint64_t sampleEveryUs = 10000, nextSampleUs = 0;

  void advanceTo(uint64_t t);
  double lineSensorCoverage(size_t i) const;
  Vec2 sensorPosition(double x, double y) const;
  double distanceReading(size_t i) const;  // 米，没有回波时为 +inf
  double lineOffset() const;                // 传感器排到最近中心线的距离

  void sync(Board& b) override { advanceTo(b.now()); }
  bool analogInput(Board& b, int pin, double& volts) override;
  bool digitalInput(Board& b, int pin, int& level) override;
  bool pulseIn(Board& b, int pin, int state, uint64_t timeoutUs, uint64_t& durationUs) override;
  void outputChanged(Board& b, int pin) override;
  ImuSample imu(Board& b) override;
  bool temperature(Board& b, double& celsius) override;
  std::mt19937 rng{368};

 private:
  void step();
  double motorCommand(const MotorConfig& m) const;
  void encoderEdges(const EncoderConfig& e, long long& index, double theta, uint64_t t0);
  long long encIndexL = 0, encIndexR = 0;
  std::vector<uint64_t> trigRise, trigFall;
  std::normal_distribution<double> gauss{0.0, 1.0};
};

}  // namespace fwsim
#endif
