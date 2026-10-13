#include "navigator.h"
#include <math.h>

static float clamp1(float v) { return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v); }

static MotorCmd drive(float left, float right) {
  MotorCmd m;
  m.left = clamp1(left);
  m.right = clamp1(right);
  return m;
}

Navigator::Navigator(const NavConfig& cfg) : cfg_(cfg), pid_(1.2f, 0.0f, 0.06f, 1.0f) {}

void Navigator::start() {
  route_.rewind();
  crossings_ = 0;
  pid_.reset();
  state_ = NavState::Following;
  timer_ = lost_time_ = 0.0f;
}

void Navigator::stop() { state_ = NavState::Idle; }

bool Navigator::running() const {
  return state_ != NavState::Idle && state_ != NavState::Finished && state_ != NavState::Failed;
}

void Navigator::enter(NavState s, const NavInput& in) {
  state_ = s;
  timer_ = lost_time_ = 0.0f;
  mark_dist_ = in.distance_m;
  mark_heading_ = in.heading_rad;
  left_line_ = false;
  if (s == NavState::Following) pid_.reset();
}

MotorCmd Navigator::update(const NavInput& in) {
  timer_ += in.dt;
  bool blocked = in.obstacle_m > 0.0f && in.obstacle_m < cfg_.obstacle_stop_m;
  if (blocked && (state_ == NavState::Following || state_ == NavState::Crossing || state_ == NavState::Lost)) {
    resume_ = state_;
    state_ = NavState::Obstacle;
    return MotorCmd();
  }
  switch (state_) {
    case NavState::Following: return follow(in);
    case NavState::Turning: return turn(in);
    case NavState::Lost: return search(in);
    case NavState::Crossing:
      if (in.distance_m - mark_dist_ >= cfg_.cross_distance_m) {
        enter(pending_ == Turn::Straight ? NavState::Following : NavState::Turning, in);
        return MotorCmd();
      }
      return drive(cfg_.base_speed * 0.6f, cfg_.base_speed * 0.6f);
    case NavState::Obstacle:
      if (in.obstacle_m <= 0.0f || in.obstacle_m > cfg_.obstacle_clear_m) {
        state_ = resume_;
        pid_.reset();
      }
      return MotorCmd();
    default: return MotorCmd();
  }
}

MotorCmd Navigator::follow(const NavInput& in) {
  if (in.line.intersection && timer_ > cfg_.intersection_guard_s) {
    crossings_++;
    pending_ = route_.next();
    enter(pending_ == Turn::Stop ? NavState::Finished : NavState::Crossing, in);
    return MotorCmd();
  }
  if (in.line.on_line) {
    lost_time_ = 0.0f;
  } else if ((lost_time_ += in.dt) > cfg_.lost_timeout_s) {
    enter(NavState::Lost, in);
    return MotorCmd();
  }
  float steer = pid_.update(in.line.position, in.dt);
  // slow down in sharp curves
  float base = cfg_.base_speed * (1.0f - 0.4f * fabsf(in.line.position));
  return drive(base + steer, base - steer);
}

MotorCmd Navigator::turn(const NavInput& in) {
  if (!in.line.on_line) left_line_ = true;
  float turned = fabsf(in.heading_rad - mark_heading_);
  float need = pending_ == Turn::UTurn ? cfg_.uturn_min_rad : cfg_.turn_min_rad;
  bool centered = in.line.on_line && fabsf(in.line.position) < 0.4f;
  if ((left_line_ || turned >= need) && centered) {
    enter(NavState::Following, in);
    return MotorCmd();
  }
  if (timer_ > cfg_.turn_timeout_s) {
    enter(NavState::Lost, in);
    return MotorCmd();
  }
  float s = cfg_.turn_speed;
  return pending_ == Turn::Left ? drive(-s, s) : drive(s, -s);
}

MotorCmd Navigator::search(const NavInput& in) {
  if (in.line.on_line) {
    enter(NavState::Following, in);
    return MotorCmd();
  }
  if (timer_ > cfg_.search_timeout_s) {
    enter(NavState::Failed, in);
    return MotorCmd();
  }
  float dir = in.line.position >= 0.0f ? 1.0f : -1.0f;
  float s = cfg_.turn_speed * 0.7f;
  return drive(dir * s, -dir * s);
}

const char* Navigator::state_name(NavState s) {
  static const char* const names[] = {"IDLE", "FOLLOWING", "CROSSING", "TURNING",
                                      "OBSTACLE", "LOST", "FINISHED", "FAILED"};
  return names[static_cast<uint8_t>(s)];
}
