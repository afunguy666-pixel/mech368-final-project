// Virtual course (track lines, obstacles) and virtual hardware attached to the
// simulated boards: a differential-drive vehicle for the robot board and a
// scripted hand (tilt) for the remote board. Every number here is an
// assumption to be replaced by bench measurements.
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
struct Pose2 { double x = 0, y = 0, yaw = 0; };  // metres, radians (CCW)

// ----------------------------------------------------------------- world ---
struct World {
  struct Line { std::vector<Vec2> pts; double width = 0.019; };
  struct Box { Vec2 lo, hi; std::string name; };
  struct Circle { Vec2 c; double r = 0; std::string name; };
  struct Wall { Vec2 a, b; std::string name; };
  // Temperature bump: ambient + delta * exp(-(d / radius)^2).
  struct Heat { Vec2 c; double radius = 0, delta = 0; std::string name; };

  double width = 3.0, height = 2.0;
  Pose2 start;
  std::vector<Line> lines;
  std::vector<Box> boxes;
  std::vector<Circle> circles;
  std::vector<Wall> walls;
  double ambient = 22.0;  // deg C
  std::vector<Heat> heat;
  std::string source = "(built-in empty world)";

  static World load(const std::string& path);
  // Distance from p to the nearest line centreline (inf when there is none).
  double lineCentreDistance(Vec2 p) const;
  // Fraction 0..1 of a sensor spot of the given radius that covers a line.
  double lineCoverage(Vec2 p, double spotRadius) const;
  // Distance along a ray to the first obstacle, or +inf beyond maxRange.
  double raycast(Vec2 origin, double yaw, double maxRange) const;
  bool collides(Vec2 centre, double radius, std::string* what) const;
  double lineLength() const;
  double temperature(Vec2 p) const;  // true air temperature, deg C
};

// --------------------------------------------------------------- vehicle ---
enum class Driver { None, PwmDir, InIn, EnInIn };
struct MotorConfig {
  Driver driver = Driver::None;
  int pwm = -1, dir = -1, in1 = -1, in2 = -1, en = -1;
  int sleep = -1;  // optional driver enable (e.g. DRV8833 nSLEEP): outputs off while LOW
  bool invert = false;
};
struct EncoderConfig { int a = -1, b = -1; double ppr = 0; bool invert = false; };
struct LineSensorConfig {
  std::string name;
  int pin = -1;
  bool analog = true;
  double x = 0, y = 0;                  // floor spot after loading, x forward, y left (m)
  double onVolts = 2.8, offVolts = 0.4; // analog output over line / floor
  int onLevel = 1;                      // digital output level over the line
  double spot = 0.004;                  // sensing spot radius (m)
};
struct DistanceConfig {
  // IRProximity: reflective IR pair (e.g. LTE-4206 + SFH 213) through a
  // comparator (LM393): a digital output that switches when an object is
  // within maxRange anywhere in a beam of half-angle halfFov.
  enum Kind { Ultrasonic, AnalogIR, IRProximity } kind = Ultrasonic;
  std::string name;
  int trig = -1, echo = -1, pin = -1;
  double x = 0, y = 0, yaw = 0;  // mount pose (m, rad)
  double minRange = 0.02, maxRange = 2.0, noise = 0.002;
  double halfFov = 0.0;          // rad (IRProximity)
  int onLevel = 0;               // IRProximity output level with an object in range
};
struct BatteryConfig {
  int pin = -1;
  double divider = 3.0;  // V_battery / V_pin
  double volts = 7.4, drainPerMinute = 0.0;
};
struct VehicleConfig {
  double trackWidth = 0.10, wheelRadius = 0.020, bodyRadius = 0.07;
  double bodyOffset = 0.0;  // collision circle centre ahead of the wheel axle (m)
  double noseDown = 0.0;    // static chassis pitch on its supports (rad)
  struct PlanImage {
    std::string png;
    Vec2 lo, hi;            // ground-plane bounds relative to the axle (m)
  } plan;
  double maxWheelSpeed = 30.0;    // rad/s, no-load, full duty, nominal voltage
  double timeConstant = 0.08;     // s, first-order motor response
  double deadband = 0.06;         // |duty| below this does not move the wheel
  double nominalVolts = 7.4;
  double leftScale = 1.0, rightScale = 1.0;  // effective wheel radius factors
  double gyroBiasDps = 0.0, gyroNoiseDps = 0.0;
  double tempTau = 2.0, tempNoise = 0.05, tempOffset = 0.0;  // HS300x model
  MotorConfig left, right;
  EncoderConfig encLeft, encRight;
  std::vector<LineSensorConfig> lineSensors;
  std::vector<DistanceConfig> distance;
  BatteryConfig battery;
  std::string source = "(built-in default vehicle)";

  static VehicleConfig load(const std::string& path);
};
int parsePin(const std::string& text);

// Scripted inputs any board can have (gestures, proximity, hand tilt).
class ScriptedIO : public BoardIO {
 public:
  std::vector<std::pair<uint64_t, int>> gestures;          // time, GESTURE_*
  std::vector<std::pair<uint64_t, Vec2>> tilt;             // time, (roll, pitch) deg
  int proximityValue = 255;

  bool gestureDue(Board& b) override;
  int takeGesture(Board& b) override;
  int proximity(Board&) override { return proximityValue; }
  ImuSample imu(Board& b) override;  // hand-held board: tilt only
  Vec2 tiltAt(uint64_t t) const;
};

class Vehicle : public ScriptedIO {
 public:
  Vehicle(const VehicleConfig& config, const World& world);

  VehicleConfig cfg;
  const World& world;
  Board* board = nullptr;

  Pose2 pose;
  double omegaL = 0, omegaR = 0;   // wheel rad/s
  // External rollers for the unpowered measurement fixture, rad/s.
  bool handRolled = false;
  double handOmegaL = 0, handOmegaR = 0;
  double thetaL = 0, thetaR = 0;   // wheel angle, rad
  double dutyL = 0, dutyR = 0;     // signed motor command -1..1
  double speed = 0, yawRate = 0, accel = 0;
  double batteryVolts = 7.4;
  double sensedTemp = 0;  // lagged temperature at the sensor, deg C
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
  double distanceReading(size_t i) const;  // metres, +inf if no echo
  double lineOffset() const;                // sensor bar to nearest centreline

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
