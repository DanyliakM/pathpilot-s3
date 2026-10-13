#pragma once
#include <stdint.h>

// differential drive dead reckoning, heading is not wrapped
class Odometry {
 public:
  Odometry(float wheel_diameter_m, uint16_t ticks_per_rev, float track_m);
  void update(int32_t left_ticks, int32_t right_ticks);
  void reset();
  float distance() const { return dist_; }
  float heading() const { return heading_; }
  float x() const { return x_; }
  float y() const { return y_; }

 private:
  float m_per_tick_, track_;
  float dist_ = 0.0f, heading_ = 0.0f, x_ = 0.0f, y_ = 0.0f;
};
