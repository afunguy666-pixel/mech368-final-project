// 仅供模拟器演示 —— 不是机器人固件，也不属于课程构建的一部分。
// 演示控制电机、编码器、巡线传感器、超声波传感器、APDS 手势、IMU 和电池分压的
// 普通 Arduino 代码如何在固件模拟器中运行。引脚与 sim/vehicles/example.vehicle
// 一致（均为占位值）。本程序**没有**使用 BLE 链路或 MotionSafety 闸门；
// 真正的自主模式必须和机器人共用同一个经过安全闸门的输出通路。
//
// 行为：等待向上的手势后开始，用编码器速度闭环沿胶带巡线；前方 20 cm 内有障碍物
// 就停车；向下的手势停车；电量低时点亮红色 LED。
// 建图：在 Serial 上发送 "pose X Y 航向角(度)" 给出起始位姿。之后程序跟踪自身位姿
// （编码器测距离 + 陀螺仪测航向，等待期间校准陀螺仪零偏），
// 并每 150 ms 打印一次 "MAP x y 温度"。
#include <Arduino.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_HS300x.h>

const int LEFT_PWM = D5, LEFT_DIR = D4, RIGHT_PWM = D6, RIGHT_DIR = D7;
const int LEFT_ENC_A = D2, LEFT_ENC_B = D3, RIGHT_ENC_A = D8, RIGHT_ENC_B = D9;
const int LINE_LEFT = A0, LINE_CENTRE = A1, LINE_RIGHT = A2;
const int TRIG = D10, ECHO = D11, BATTERY = A7;

const float WHEEL_RADIUS = 0.020f, TRACK = 0.10f;
const float COUNTS_PER_REV = 720.0f;   // 360 ppr，取 A 相的上升沿和下降沿
const float MAX_WHEEL_SPEED = 30.0f;   // rad/s，用于前馈项
const float CRUISE = 0.25f;            // m/s
const float STOP_DISTANCE = 0.20f, CLEAR_DISTANCE = 0.30f;
const float LOW_BATTERY_VOLTS = 6.8f;

volatile long leftCount = 0, rightCount = 0;
void leftEncoderIsr() { leftCount += digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B) ? -1 : 1; }
void rightEncoderIsr() { rightCount += digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B) ? -1 : 1; }

enum State { WAITING, FOLLOWING, BLOCKED, STOPPED };
State state = WAITING;
const char* stateName(State s) {
  switch (s) {
    case WAITING: return "等待";
    case FOLLOWING: return "巡线";
    case BLOCKED: return "受阻";
    case STOPPED: return "停止";
  }
  return "?";
}

float targetLeft = 0, targetRight = 0;  // 车轮 rad/s
float integralLeft = 0, integralRight = 0;
long lastLeftCount = 0, lastRightCount = 0;
float linePosition = 0, lastLinePosition = 0;
float obstacle = -1;                     // 米，-1 = 量程内没有物体
float odoX = 0, odoY = 0, odoHeading = 0, gyroHeading = 0;
bool lowBattery = false;
bool mapping = false;                    // 由串口命令 "pose" 设置
float poseX = 0, poseY = 0, poseHeading = 0;  // 编码器测距离 + 陀螺仪测航向
float gyroBias = 0, biasSum = 0;
long biasCount = 0;
unsigned long lastMap = 0, lastGyroUs = 0;
unsigned long lastControl = 0, lastLine = 0, lastSonar = 0, lastBattery = 0, lastReport = 0;

void drive(int pwmPin, int dirPin, float duty) {
  duty = constrain(duty, -1.0f, 1.0f);
  digitalWrite(dirPin, duty >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, static_cast<int>(fabsf(duty) * 255.0f + 0.5f));
}

void setState(State next) {
  if (next == state) return;
  Serial.print("状态：");
  Serial.print(stateName(state));
  Serial.print(" -> ");
  Serial.println(stateName(next));
  state = next;
  digitalWrite(LEDG, state == FOLLOWING ? LOW : HIGH);  // RGB LED 为低电平点亮
  digitalWrite(LEDB, state == WAITING ? LOW : HIGH);
}

// 加权的线位置：+1 = 线在左传感器下方，-1 = 在右侧。
void readLine() {
  const float left = analogRead(LINE_LEFT) / 1023.0f;
  const float centre = analogRead(LINE_CENTRE) / 1023.0f;
  const float right = analogRead(LINE_RIGHT) / 1023.0f;
  const float floorLevel = 0.15f;  // 地面上约 0.4 V
  const float l = max(0.0f, left - floorLevel), c = max(0.0f, centre - floorLevel),
              r = max(0.0f, right - floorLevel);
  const float sum = l + c + r;
  lastLinePosition = linePosition;
  if (sum > 0.05f) linePosition = (l - r) / sum;
  else linePosition = linePosition >= 0 ? 1.5f : -1.5f;  // 丢线：继续朝线的方向转
}

void readSonar() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  const unsigned long echo = pulseIn(ECHO, HIGH, 12000UL);  // 最远约 2 m
  obstacle = echo ? echo * 343.0f / 2.0f / 1e6f : -1.0f;
}

void speedControl(float dt) {
  noInterrupts();
  const long l = leftCount, r = rightCount;
  interrupts();
  const float wl = (l - lastLeftCount) / COUNTS_PER_REV * TWO_PI / dt;
  const float wr = (r - lastRightCount) / COUNTS_PER_REV * TWO_PI / dt;
  const float dl = (l - lastLeftCount) / COUNTS_PER_REV * TWO_PI * WHEEL_RADIUS;
  const float dr = (r - lastRightCount) / COUNTS_PER_REV * TWO_PI * WHEEL_RADIUS;
  lastLeftCount = l;
  lastRightCount = r;
  const float ds = (dl + dr) / 2, dyaw = (dr - dl) / TRACK;
  odoX += ds * cosf(odoHeading + dyaw / 2);
  odoY += ds * sinf(odoHeading + dyaw / 2);
  odoHeading += dyaw;
  poseX += ds * cosf(poseHeading);
  poseY += ds * sinf(poseHeading);

  const float kp = 0.02f, ki = 0.15f;
  integralLeft = constrain(integralLeft + (targetLeft - wl) * dt, -2.0f, 2.0f);
  integralRight = constrain(integralRight + (targetRight - wr) * dt, -2.0f, 2.0f);
  if (targetLeft == 0 && targetRight == 0) {
    integralLeft = integralRight = 0;
    drive(LEFT_PWM, LEFT_DIR, 0);
    drive(RIGHT_PWM, RIGHT_DIR, 0);
    return;
  }
  drive(LEFT_PWM, LEFT_DIR, targetLeft / MAX_WHEEL_SPEED + kp * (targetLeft - wl) + ki * integralLeft);
  drive(RIGHT_PWM, RIGHT_DIR, targetRight / MAX_WHEEL_SPEED + kp * (targetRight - wr) + ki * integralRight);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(LEFT_ENC_A, INPUT);
  pinMode(LEFT_ENC_B, INPUT);
  pinMode(RIGHT_ENC_A, INPUT);
  pinMode(RIGHT_ENC_B, INPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, LOW);
  drive(LEFT_PWM, LEFT_DIR, 0);
  drive(RIGHT_PWM, RIGHT_DIR, 0);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderIsr, CHANGE);
  if (!IMU.begin()) Serial.println("IMU 初始化失败");
  if (!APDS.begin()) Serial.println("APDS 初始化失败");
  if (!HS300x.begin()) Serial.println("HS300x 初始化失败");
  Serial.println("巡线程序就绪：向上挥手开始，向下挥手停止");
}

void loop() {
  const unsigned long now = millis();

  if (APDS.gestureAvailable()) {
    const int g = APDS.readGesture();
    if (g == GESTURE_UP && (state == WAITING || state == STOPPED)) {
      Serial.println("手势 UP：开始");
      setState(FOLLOWING);
    } else if (g == GESTURE_DOWN) {
      Serial.println("手势 DOWN：停止");
      setState(STOPPED);
    }
  }

  // "pose X Y 航向角(度)"：已知的起始位姿；收到后开启建图输出。
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("pose ")) {
      line = line.substring(5);
      const int a = line.indexOf(' '), b = line.indexOf(' ', a + 1);
      if (a > 0 && b > a) {
        poseX = line.substring(0, a).toFloat();
        poseY = line.substring(a + 1, b).toFloat();
        poseHeading = radians(line.substring(b + 1).toFloat());
        mapping = true;
        Serial.println("已开启建图");
      }
    }
  }

  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    // 按距上次读取的实际时间积分，而不是固定的 10 ms：
    // 阻塞调用（读温度、pulseIn）会让主循环跳过样本。
    const unsigned long nowUs = micros();
    const float dt = lastGyroUs ? (nowUs - lastGyroUs) / 1e6f : 0.01f;
    lastGyroUs = nowUs;
    if (state == WAITING) {  // 静止时：估计陀螺仪零偏
      biasSum += gz;
      gyroBias = biasSum / ++biasCount;
    } else {
      gyroHeading += (gz - gyroBias) * dt;  // 度
      poseHeading += radians((gz - gyroBias) * dt);
    }
  }

  if (now - lastSonar >= 60) {
    lastSonar = now;
    readSonar();
    if (state == FOLLOWING && obstacle > 0 && obstacle < STOP_DISTANCE) {
      Serial.print("前方障碍物距离 ");
      Serial.print(obstacle * 100, 1);
      Serial.println(" cm：停车");
      setState(BLOCKED);
    } else if (state == BLOCKED && (obstacle < 0 || obstacle > CLEAR_DISTANCE)) {
      Serial.println("前方已畅通：继续");
      setState(FOLLOWING);
    }
  }

  if (now - lastLine >= 5) {
    lastLine = now;
    readLine();
    if (state == FOLLOWING) {
      const float error = linePosition;
      const float omega = 6.0f * error + 0.25f * (linePosition - lastLinePosition) / 0.005f;
      const float v = CRUISE * (1.0f - 0.5f * min(1.0f, fabsf(error)));
      targetLeft = (v - omega * TRACK / 2) / WHEEL_RADIUS;
      targetRight = (v + omega * TRACK / 2) / WHEEL_RADIUS;
    } else {
      targetLeft = targetRight = 0;
    }
  }

  if (now - lastControl >= 20) {
    speedControl((now - lastControl) / 1000.0f);
    lastControl = now;
  }

  if (mapping && state == FOLLOWING && now - lastMap >= 150) {
    lastMap = now;
    const float celsius = HS300x.readTemperature();
    Serial.print("MAP ");
    Serial.print(poseX, 3);
    Serial.print(' ');
    Serial.print(poseY, 3);
    Serial.print(' ');
    Serial.println(celsius, 2);
  }

  if (now - lastBattery >= 500) {
    lastBattery = now;
    const float volts = analogRead(BATTERY) * 3.3f / 1023.0f * 3.0f;
    if (!lowBattery && volts < LOW_BATTERY_VOLTS) {
      lowBattery = true;
      Serial.print("电量低：");
      Serial.print(volts, 2);
      Serial.println(" V");
      digitalWrite(LEDR, LOW);
    }
  }

  if (now - lastReport >= 1000) {
    lastReport = now;
    Serial.print(stateName(state));
    Serial.print(" 里程计=(");
    Serial.print(odoX, 3);
    Serial.print(", ");
    Serial.print(odoY, 3);
    Serial.print(") m 航向 编码器=");
    Serial.print(degrees(odoHeading), 1);
    Serial.print(" 陀螺仪=");
    Serial.print(gyroHeading, 1);
    Serial.print(" 度 赛道=");
    Serial.print(linePosition, 2);
    Serial.print(" 超声波=");
    Serial.println(obstacle, 2);
  }
}
