#include "odometry.h"
#include <math.h>

Odometry::Odometry(float wheel_diameter_m, uint16_t ticks_per_rev, float track_m)
    : m_per_tick_(float(M_PI) * wheel_diameter_m / float(ticks_per_rev)), track_(track_m) {}

void Odometry::update(int32_t left_ticks, int32_t right_ticks) {
  float dl = float(left_ticks) * m_per_tick_;
  float dr = float(right_ticks) * m_per_tick_;
  float d = 0.5f * (dl + dr);
  float mid = heading_ + 0.5f * (dr - dl) / track_;
  x_ += d * cosf(mid);
  y_ += d * sinf(mid);
  heading_ += (dr - dl) / track_;
  dist_ += d;
}

void Odometry::reset() { dist_ = heading_ = x_ = y_ = 0.0f; }
