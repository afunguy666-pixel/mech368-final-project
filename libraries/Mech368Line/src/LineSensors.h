// IR reflectance line sensor array: calibration, normalisation and line
// position. Works with
//   - analog sensors: LTE-4206 emitter + LTR-4206 phototransistor read by
//     analogRead() (continuous position, best for PD control), or
//   - digital sensors: the same pair through an LM393 comparator, read by
//     digitalRead() (or analogRead() of the comparator output).
//
// The position estimate is a weighted average of the sensor positions, the
// same idea as Pololu's QTRSensors readLineBlack(): each sensor is first
// scaled to 0 (white floor) .. 1 (black tape) with its own calibrated
// minimum and maximum, small values are treated as noise, and when no sensor
// sees the line the last known side is reported so the robot keeps turning
// toward where the line went. This is an independent implementation.
//
// Positions are lateral offsets in metres in the robot frame, positive to the
// robot's LEFT. A positive line position means the line is left of centre,
// so the robot must turn left (positive omega).
#ifndef MECH368_LINE_SENSORS_H
#define MECH368_LINE_SENSORS_H

#include <math.h>
#include <stdint.h>

namespace mech368 {
namespace line {

const int kMaxLineSensors = 8;

struct LineReading {
  bool onLine = false;      // at least one sensor clearly over the tape
  float position = 0.0f;    // m, + = line left of centre; extrapolated when lost
  float lastSeen = 0.0f;    // last position measured while on the line
  int blackCount = 0;       // sensors at or above the black threshold
  uint8_t blackMask = 0;    // bit i set: sensor i is black
  bool leftOuterBlack = false, rightOuterBlack = false;
  float peak = 0.0f;        // largest normalised value
  float value[kMaxLineSensors] = {};  // normalised 0 (white) .. 1 (black)
};

class LineSensorArray {
 public:
  // Thresholds on normalised values (0..1).
  float noiseFloor = 0.10f;      // ignored in the weighted average
  float blackThreshold = 0.50f;  // sensor counts as "black" (also onLine)
  float lostMargin = 0.015f;     // extrapolated position beyond the outer sensor (m)
  bool blackIsHigh = true;       // raw reading rises over black tape

  // y: lateral position of each sensor (m, + left), in any order.
  bool configure(int count, const float* y) {
    if (count < 2 || count > kMaxLineSensors) return false;
    count_ = count;
    left_ = right_ = 0;
    for (int i = 0; i < count; ++i) {
      y_[i] = y[i];
      if (y[i] > y_[left_]) left_ = i;
      if (y[i] < y_[right_]) right_ = i;
    }
    resetCalibration();
    lastSeen_ = 0.0f;
    return true;
  }
  int count() const { return count_; }
  float sensorY(int i) const { return y_[i]; }
  int leftOuter() const { return left_; }
  int rightOuter() const { return right_; }

  // ---- calibration (analog) ----
  // Sweep every sensor over both the tape and the floor while calling
  // calibrate() with raw readings; the extremes become 0 and 1.
  void resetCalibration() {
    for (int i = 0; i < kMaxLineSensors; ++i) {
      min_[i] = 0xFFFF;
      max_[i] = 0;
    }
  }
  void calibrate(const uint16_t* raw) {
    for (int i = 0; i < count_; ++i) {
      if (raw[i] < min_[i]) min_[i] = raw[i];
      if (raw[i] > max_[i]) max_[i] = raw[i];
    }
  }
  // Fixed calibration from bench readings over the floor and the tape.
  void setCalibration(int i, uint16_t floorRaw, uint16_t tapeRaw) {
    min_[i] = floorRaw < tapeRaw ? floorRaw : tapeRaw;
    max_[i] = floorRaw < tapeRaw ? tapeRaw : floorRaw;
  }
  uint16_t calibrationMin(int i) const { return min_[i]; }
  uint16_t calibrationMax(int i) const { return max_[i]; }
  // True when every sensor saw a spread of at least minSpan raw counts. A
  // sensor that never left the floor (or the tape) cannot be scaled.
  bool calibrationValid(uint16_t minSpan) const {
    for (int i = 0; i < count_; ++i)
      if (max_[i] < min_[i] || max_[i] - min_[i] < minSpan) return false;
    return true;
  }
  float normalize(int i, uint16_t raw) const {
    if (max_[i] <= min_[i]) return 0.0f;
    float v = (static_cast<float>(raw) - min_[i]) / static_cast<float>(max_[i] - min_[i]);
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return blackIsHigh ? v : 1.0f - v;
  }

  // ---- reading ----
  LineReading readRaw(const uint16_t* raw) {
    float v[kMaxLineSensors];
    for (int i = 0; i < count_; ++i) v[i] = normalize(i, raw[i]);
    return read(v);
  }
  // Comparator outputs already converted to "this sensor sees black".
  LineReading readDigital(const bool* black) {
    float v[kMaxLineSensors];
    for (int i = 0; i < count_; ++i) v[i] = black[i] ? 1.0f : 0.0f;
    return read(v);
  }
  // Normalised values 0 (white) .. 1 (black).
  LineReading read(const float* v) {
    LineReading r;
    float sum = 0.0f, weighted = 0.0f;
    for (int i = 0; i < count_; ++i) {
      const float x = v[i] < 0.0f ? 0.0f : (v[i] > 1.0f ? 1.0f : v[i]);
      r.value[i] = x;
      if (x > r.peak) r.peak = x;
      if (x >= blackThreshold) {
        ++r.blackCount;
        r.blackMask |= static_cast<uint8_t>(1u << i);
      }
      if (x >= noiseFloor) {
        sum += x;
        weighted += x * y_[i];
      }
    }
    r.leftOuterBlack = v[left_] >= blackThreshold;
    r.rightOuterBlack = v[right_] >= blackThreshold;
    r.onLine = r.blackCount > 0 && sum > 0.0f;
    if (r.onLine) {
      r.position = weighted / sum;
      lastSeen_ = r.position;
    } else {
      // Lost: report "past the outer sensor" on the side the line was last seen.
      r.position = lastSeen_ >= 0.0f ? y_[left_] + lostMargin : y_[right_] - lostMargin;
    }
    r.lastSeen = lastSeen_;
    return r;
  }
  void setLastSeen(float position) { lastSeen_ = position; }

 private:
  int count_ = 0, left_ = 0, right_ = 0;
  float y_[kMaxLineSensors] = {};
  uint16_t min_[kMaxLineSensors] = {}, max_[kMaxLineSensors] = {};
  float lastSeen_ = 0.0f;
};

}  // namespace line
}  // namespace mech368
#endif
