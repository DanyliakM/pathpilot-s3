#pragma once

class Pid {
 public:
  Pid(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float limit = 1.0f);
  void set_gains(float kp, float ki, float kd);
  float update(float error, float dt);
  void reset();
  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float kd() const { return kd_; }

 private:
  float kp_, ki_, kd_, limit_;
  float integral_ = 0.0f;
  float prev_error_ = 0.0f;
  bool first_ = true;
};
