#pragma once
#include <stddef.h>
#include <stdint.h>

constexpr size_t kLineSensors = 5;

struct LineReading {
  float position = 0.0f;  // -1 = line under leftmost sensor, +1 = rightmost
  float level[kLineSensors] = {};
  uint8_t active = 0;
  bool on_line = false;
  bool intersection = false;
};

class LineTracker {
 public:
  LineTracker() { reset_calibration(); }
  void reset_calibration();
  void calibrate(const uint16_t* raw);
  void set_calibration(const uint16_t* min, const uint16_t* max);
  bool calibrated() const;
  LineReading process(const uint16_t* raw);
  const uint16_t* cal_min() const { return min_; }
  const uint16_t* cal_max() const { return max_; }

 private:
  uint16_t min_[kLineSensors];
  uint16_t max_[kLineSensors];
  float last_pos_ = 0.0f;
  float threshold_ = 0.5f;
};
