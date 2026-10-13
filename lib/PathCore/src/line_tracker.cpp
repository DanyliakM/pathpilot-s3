#include "line_tracker.h"

static constexpr int32_t kMinSpan = 200;
static constexpr float kNoiseFloor = 0.15f;

void LineTracker::reset_calibration() {
  for (size_t i = 0; i < kLineSensors; ++i) {
    min_[i] = UINT16_MAX;
    max_[i] = 0;
  }
}

void LineTracker::calibrate(const uint16_t* raw) {
  for (size_t i = 0; i < kLineSensors; ++i) {
    if (raw[i] < min_[i]) min_[i] = raw[i];
    if (raw[i] > max_[i]) max_[i] = raw[i];
  }
}

void LineTracker::set_calibration(const uint16_t* min, const uint16_t* max) {
  for (size_t i = 0; i < kLineSensors; ++i) {
    min_[i] = min[i];
    max_[i] = max[i];
  }
}

bool LineTracker::calibrated() const {
  for (size_t i = 0; i < kLineSensors; ++i)
    if (int32_t(max_[i]) - int32_t(min_[i]) < kMinSpan) return false;
  return true;
}

LineReading LineTracker::process(const uint16_t* raw) {
  LineReading r;
  float sum = 0.0f, weighted = 0.0f;
  for (size_t i = 0; i < kLineSensors; ++i) {
    int32_t span = int32_t(max_[i]) - int32_t(min_[i]);
    float n = span > 0 ? float(int32_t(raw[i]) - int32_t(min_[i])) / float(span) : 0.0f;
    n = n < 0.0f ? 0.0f : (n > 1.0f ? 1.0f : n);
    r.level[i] = n;
    if (n >= threshold_) r.active++;
    if (n < kNoiseFloor) continue;
    float w = 2.0f * float(i) / float(kLineSensors - 1) - 1.0f;
    sum += n;
    weighted += n * w;
  }
  r.on_line = r.active > 0;
  r.intersection = r.active >= kLineSensors - 1;
  if (r.on_line && sum > 0.0f) {
    r.position = weighted / sum;
    last_pos_ = r.position;
  } else {
    // line lost: saturate towards the side where it was seen last
    r.position = last_pos_ > 0.05f ? 1.0f : (last_pos_ < -0.05f ? -1.0f : 0.0f);
  }
  return r;
}
