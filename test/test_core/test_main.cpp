#include <math.h>
#include <unity.h>
#include "line_tracker.h"
#include "navigator.h"
#include "odometry.h"
#include "pid.h"
#include "route.h"

void setUp() {}
void tearDown() {}

// helpers

static LineTracker calibrated_tracker() {
  LineTracker t;
  uint16_t mn[kLineSensors] = {200, 200, 200, 200, 200};
  uint16_t mx[kLineSensors] = {3200, 3200, 3200, 3200, 3200};
  t.set_calibration(mn, mx);
  return t;
}

static LineReading line_at(float pos, bool on = true, bool junction = false) {
  LineReading r;
  r.position = pos;
  r.on_line = on;
  r.active = junction ? 5 : (on ? 1 : 0);
  r.intersection = junction;
  return r;
}

static NavInput input(const LineReading& l, float dist = 0, float heading = 0, float obstacle = -1) {
  return NavInput{l, obstacle, dist, heading, 0.01f};
}

// pid

void test_pid_proportional_and_clamp() {
  Pid p(2.0f, 0.0f, 0.0f, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.4f, p.update(0.2f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, p.update(5.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-5, -1.0f, p.update(-5.0f, 0.01f));
}

void test_pid_anti_windup() {
  Pid p(0.0f, 1.0f, 0.0f, 1.0f);
  for (int i = 0; i < 1000; ++i) p.update(1.0f, 0.1f);
  // after a long saturation the output must recover quickly
  float out = 0;
  for (int i = 0; i < 25; ++i) out = p.update(-1.0f, 0.1f);
  TEST_ASSERT_TRUE(out < 0.0f);
}

void test_pid_derivative_reacts_to_change() {
  Pid p(0.0f, 0.0f, 0.1f, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.0f, p.update(0.0f, 0.01f));
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.5f, p.update(0.05f, 0.01f));
}

// line tracker

void test_tracker_center_left_right() {
  LineTracker t = calibrated_tracker();
  uint16_t center[] = {200, 200, 3200, 200, 200};
  uint16_t left[] = {3200, 1700, 200, 200, 200};
  uint16_t right[] = {200, 200, 200, 3200, 3200};
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, t.process(center).position);
  TEST_ASSERT_TRUE(t.process(left).position < -0.7f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.75f, t.process(right).position);
}

void test_tracker_lost_keeps_last_side() {
  LineTracker t = calibrated_tracker();
  uint16_t right[] = {200, 200, 200, 3200, 200};
  uint16_t none[] = {210, 190, 220, 205, 200};
  t.process(right);
  LineReading r = t.process(none);
  TEST_ASSERT_FALSE(r.on_line);
  TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.0f, r.position);
}

void test_tracker_intersection() {
  LineTracker t = calibrated_tracker();
  uint16_t all[] = {3100, 3000, 3200, 2900, 3100};
  LineReading r = t.process(all);
  TEST_ASSERT_TRUE(r.intersection);
  TEST_ASSERT_EQUAL_UINT8(5, r.active);
}

void test_tracker_calibration() {
  LineTracker t;
  TEST_ASSERT_FALSE(t.calibrated());
  uint16_t white[] = {300, 310, 290, 305, 300};
  uint16_t black[] = {3000, 2900, 3100, 3050, 2950};
  t.calibrate(white);
  t.calibrate(black);
  TEST_ASSERT_TRUE(t.calibrated());
  TEST_ASSERT_EQUAL_UINT16(290, t.cal_min()[2]);
  TEST_ASSERT_EQUAL_UINT16(3100, t.cal_max()[2]);
}

// route

void test_route_parse_and_sequence() {
  Route r;
  TEST_ASSERT_TRUE(r.parse("s, l-r u"));
  TEST_ASSERT_EQUAL_STRING("SLRU", r.str());
  TEST_ASSERT_TRUE(r.next() == Turn::Straight);
  TEST_ASSERT_TRUE(r.next() == Turn::Left);
  TEST_ASSERT_TRUE(r.next() == Turn::Right);
  TEST_ASSERT_TRUE(r.next() == Turn::UTurn);
  TEST_ASSERT_TRUE(r.next() == Turn::Stop);
  r.rewind();
  TEST_ASSERT_TRUE(r.next() == Turn::Straight);
}

void test_route_rejects_invalid_and_keeps_old() {
  Route r;
  TEST_ASSERT_TRUE(r.parse("LR"));
  TEST_ASSERT_FALSE(r.parse("LXR"));
  TEST_ASSERT_FALSE(r.parse(""));
  TEST_ASSERT_FALSE(r.parse("SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS"));  // 33 steps
  TEST_ASSERT_EQUAL_STRING("LR", r.str());
}

// odometry

void test_odometry_straight_and_spin() {
  Odometry o(0.05f, 100, 0.1f);
  o.update(100, 100);  // one wheel revolution
  TEST_ASSERT_FLOAT_WITHIN(1e-4, float(M_PI) * 0.05f, o.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-4, float(M_PI) * 0.05f, o.x());
  o.reset();
  o.update(-100, 100);  // spin in place to the left
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.0f, o.distance());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, float(M_PI) * 0.05f * 2.0f / 0.1f, o.heading());
}

// navigator

void test_nav_steers_towards_line() {
  Navigator n;
  n.start();
  MotorCmd m = n.update(input(line_at(0.5f)));
  TEST_ASSERT_TRUE(m.left > m.right);  // line on the right -> turn right
  m = n.update(input(line_at(-0.5f)));
  TEST_ASSERT_TRUE(m.right > m.left);
}

void test_nav_straight_junction() {
  Navigator n;
  n.route().parse("S");
  n.start();
  for (int i = 0; i < 30; ++i) n.update(input(line_at(0)));
  n.update(input(line_at(0, true, true), 0.10f));
  TEST_ASSERT_TRUE(n.state() == NavState::Crossing);
  TEST_ASSERT_EQUAL_UINT16(1, n.crossings());
  n.update(input(line_at(0, true, true), 0.13f));
  TEST_ASSERT_TRUE(n.state() == NavState::Crossing);
  n.update(input(line_at(0), 0.17f));
  TEST_ASSERT_TRUE(n.state() == NavState::Following);
}

void test_nav_left_turn_until_reacquired() {
  Navigator n;
  n.route().parse("L");
  n.start();
  for (int i = 0; i < 30; ++i) n.update(input(line_at(0)));
  n.update(input(line_at(0, true, true), 0.0f));
  n.update(input(line_at(0), 0.07f));
  TEST_ASSERT_TRUE(n.state() == NavState::Turning);
  MotorCmd m = n.update(input(line_at(0), 0.07f, 0.1f));  // old line still under sensors
  TEST_ASSERT_TRUE(n.state() == NavState::Turning);
  TEST_ASSERT_TRUE(m.left < 0 && m.right > 0);
  n.update(input(line_at(1.0f, false), 0.07f, 0.4f));  // leaves the old line
  n.update(input(line_at(0.1f), 0.07f, 1.4f));         // new branch centered
  TEST_ASSERT_TRUE(n.state() == NavState::Following);
}

void test_nav_finishes_after_route() {
  Navigator n;
  n.route().parse("S");
  n.start();
  for (int i = 0; i < 30; ++i) n.update(input(line_at(0)));
  n.update(input(line_at(0, true, true)));
  n.update(input(line_at(0), 0.1f));
  for (int i = 0; i < 30; ++i) n.update(input(line_at(0), 0.2f));
  MotorCmd m = n.update(input(line_at(0, true, true), 0.4f));
  TEST_ASSERT_TRUE(n.state() == NavState::Finished);
  TEST_ASSERT_FALSE(n.running());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, m.left);
}

void test_nav_obstacle_stop_and_resume() {
  Navigator n;
  n.start();
  MotorCmd m = n.update(input(line_at(0), 0, 0, 0.10f));
  TEST_ASSERT_TRUE(n.state() == NavState::Obstacle);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, m.left);
  n.update(input(line_at(0), 0, 0, 0.20f));  // inside hysteresis band
  TEST_ASSERT_TRUE(n.state() == NavState::Obstacle);
  n.update(input(line_at(0), 0, 0, -1.0f));
  TEST_ASSERT_TRUE(n.state() == NavState::Following);
}

void test_nav_lost_then_failed() {
  Navigator n;
  n.start();
  for (int i = 0; i < 100; ++i) n.update(input(line_at(1.0f, false)));
  TEST_ASSERT_TRUE(n.state() == NavState::Lost);
  for (int i = 0; i < 400; ++i) n.update(input(line_at(1.0f, false)));
  TEST_ASSERT_TRUE(n.state() == NavState::Failed);
}

// closed loop: kinematic robot must converge onto a straight line y = 0
void test_closed_loop_converges() {
  Navigator n;
  n.start();
  const float dt = 0.01f, vmax = 0.5f, track = 0.12f, bar = 0.06f;
  float x = 0, y = 0.02f, th = 0.0f;
  for (int i = 0; i < 600; ++i) {
    float e = y + bar * sinf(th);  // sensor bar offset, > 0 means line is to the right
    float pos = fmaxf(-1.0f, fminf(1.0f, e / 0.03f));
    NavInput in{line_at(pos, fabsf(e) < 0.04f), -1.0f, x, th, dt};
    MotorCmd m = n.update(in);
    float vl = m.left * vmax, vr = m.right * vmax;
    float v = 0.5f * (vl + vr);
    th += (vr - vl) / track * dt;
    x += v * cosf(th) * dt;
    y += v * sinf(th) * dt;
  }
  TEST_ASSERT_TRUE(n.state() == NavState::Following);
  TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.0f, y);
  TEST_ASSERT_TRUE(x > 0.5f);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_pid_proportional_and_clamp);
  RUN_TEST(test_pid_anti_windup);
  RUN_TEST(test_pid_derivative_reacts_to_change);
  RUN_TEST(test_tracker_center_left_right);
  RUN_TEST(test_tracker_lost_keeps_last_side);
  RUN_TEST(test_tracker_intersection);
  RUN_TEST(test_tracker_calibration);
  RUN_TEST(test_route_parse_and_sequence);
  RUN_TEST(test_route_rejects_invalid_and_keeps_old);
  RUN_TEST(test_odometry_straight_and_spin);
  RUN_TEST(test_nav_steers_towards_line);
  RUN_TEST(test_nav_straight_junction);
  RUN_TEST(test_nav_left_turn_until_reacquired);
  RUN_TEST(test_nav_finishes_after_route);
  RUN_TEST(test_nav_obstacle_stop_and_resume);
  RUN_TEST(test_nav_lost_then_failed);
  RUN_TEST(test_closed_loop_converges);
  return UNITY_END();
}
