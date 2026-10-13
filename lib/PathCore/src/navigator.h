#pragma once
#include "line_tracker.h"
#include "pid.h"
#include "route.h"

enum class NavState : uint8_t { Idle, Following, Crossing, Turning, Obstacle, Lost, Finished, Failed };

struct NavConfig {
  float base_speed = 0.55f;
  float turn_speed = 0.45f;
  float cross_distance_m = 0.06f;  // drive past the junction before turning
  float intersection_guard_s = 0.2f;
  float obstacle_stop_m = 0.15f;
  float obstacle_clear_m = 0.25f;
  float lost_timeout_s = 0.6f;
  float search_timeout_s = 3.0f;
  float turn_min_rad = 0.6f;
  float uturn_min_rad = 2.2f;
  float turn_timeout_s = 4.0f;
};

struct NavInput {
  LineReading line;
  float obstacle_m;  // <= 0 means nothing in front
  float distance_m;
  float heading_rad;
  float dt;
};

struct MotorCmd {
  float left = 0.0f;
  float right = 0.0f;
};

class Navigator {
 public:
  explicit Navigator(const NavConfig& cfg = NavConfig());
  void start();
  void stop();
  MotorCmd update(const NavInput& in);
  NavState state() const { return state_; }
  bool running() const;
  uint16_t crossings() const { return crossings_; }
  Route& route() { return route_; }
  Pid& pid() { return pid_; }
  NavConfig& config() { return cfg_; }
  static const char* state_name(NavState s);

 private:
  void enter(NavState s, const NavInput& in);
  MotorCmd follow(const NavInput& in);
  MotorCmd turn(const NavInput& in);
  MotorCmd search(const NavInput& in);

  NavConfig cfg_;
  Pid pid_;
  Route route_;
  NavState state_ = NavState::Idle;
  NavState resume_ = NavState::Following;
  Turn pending_ = Turn::Straight;
  float timer_ = 0.0f, lost_time_ = 0.0f;
  float mark_dist_ = 0.0f, mark_heading_ = 0.0f;
  bool left_line_ = false;
  uint16_t crossings_ = 0;
};
