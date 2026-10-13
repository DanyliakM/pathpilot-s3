#include "pid.h"

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

Pid::Pid(float kp, float ki, float kd, float limit) : kp_(kp), ki_(ki), kd_(kd), limit_(limit) {}

void Pid::set_gains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  reset();
}

void Pid::reset() {
  integral_ = 0.0f;
  prev_error_ = 0.0f;
  first_ = true;
}

float Pid::update(float error, float dt) {
  if (dt <= 0.0f) return 0.0f;
  integral_ += error * dt;
  // anti-windup: the integral term alone never exceeds the output limit
  if (ki_ > 0.0f) integral_ = clampf(integral_, -limit_ / ki_, limit_ / ki_);
  float deriv = first_ ? 0.0f : (error - prev_error_) / dt;
  prev_error_ = error;
  first_ = false;
  return clampf(kp_ * error + ki_ * integral_ + kd_ * deriv, -limit_, limit_);
}
