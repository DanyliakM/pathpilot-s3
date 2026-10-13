#include <Arduino.h>
#include <string.h>
#include "app.h"
#include "config.h"
#include "hal/encoders.h"
#include "hal/line_sensor.h"
#include "hal/motors.h"
#include "hal/sonar.h"
#include "odometry.h"

namespace {
QueueHandle_t g_queue = nullptr;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
Telemetry g_tel;

LineTracker g_tracker;
Navigator g_nav;
Odometry g_odom(WHEEL_DIAMETER_M, ENC_TICKS_PER_REV, WHEEL_TRACK_M);
bool g_calibrating = false;
float g_cal_time = 0.0f;

void handle(const Command& c) {
  switch (c.type) {
    case CmdType::Start:
      if (!g_calibrating) {
        g_odom.reset();
        g_nav.start();
      }
      break;
    case CmdType::Stop:
      g_nav.stop();
      g_calibrating = false;
      break;
    case CmdType::Calibrate:
      g_nav.stop();
      g_tracker.reset_calibration();
      g_cal_time = 0.0f;
      g_calibrating = true;
      break;
    case CmdType::SetRoute: g_nav.route().parse(c.text); break;
    case CmdType::SetPid: g_nav.pid().set_gains(c.a, c.b, c.c); break;
    case CmdType::SetSpeed: g_nav.config().base_speed = constrain(c.a, 0.1f, 1.0f); break;
  }
}

MotorCmd calibrate_step(const uint16_t* raw, float dt) {
  g_tracker.calibrate(raw);
  g_cal_time += dt;
  if (g_cal_time >= CAL_DURATION_S) {
    g_calibrating = false;
    motors_stop();
    line_sensor_save_cal(g_tracker.cal_min(), g_tracker.cal_max());
    return MotorCmd();
  }
  // sweep the sensor bar left and right across the line
  bool left = fmodf(g_cal_time + 0.3f, 1.2f) < 0.6f;
  MotorCmd m;
  m.left = left ? -CAL_SPEED : CAL_SPEED;
  m.right = -m.left;
  return m;
}

void publish(const LineReading& line, const MotorCmd& cmd, const uint16_t* raw, uint32_t loop_us) {
  Telemetry t;
  t.state = g_nav.state();
  t.running = g_nav.running();
  t.calibrating = g_calibrating;
  t.calibrated = g_tracker.calibrated();
  t.line = line;
  memcpy(t.raw, raw, sizeof(t.raw));
  t.obstacle_m = sonar_distance_m();
  t.distance_m = g_odom.distance();
  t.heading_deg = g_odom.heading() * 57.2958f;
  t.x_m = g_odom.x();
  t.y_m = g_odom.y();
  t.motor_l = cmd.left;
  t.motor_r = cmd.right;
  t.kp = g_nav.pid().kp();
  t.ki = g_nav.pid().ki();
  t.kd = g_nav.pid().kd();
  t.speed = g_nav.config().base_speed;
  t.crossings = g_nav.crossings();
  t.route_idx = uint8_t(g_nav.route().index());
  strncpy(t.route, g_nav.route().str(), sizeof(t.route) - 1);
  t.loop_us = loop_us;
  portENTER_CRITICAL(&g_mux);
  g_tel = t;
  portEXIT_CRITICAL(&g_mux);
}

void control_task(void*) {
  const float dt = CONTROL_PERIOD_MS / 1000.0f;
  uint16_t raw[kLineSensors];
  uint32_t sonar_ms = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    uint32_t t0 = micros();

    Command c;
    while (xQueueReceive(g_queue, &c, 0) == pdTRUE) handle(c);

    if (millis() - sonar_ms >= SONAR_PERIOD_MS) {
      sonar_ms = millis();
      sonar_trigger();
    }
    int32_t dl, dr;
    encoders_read_delta(dl, dr);
    g_odom.update(dl, dr);
    line_sensor_read(raw);
    LineReading line = g_tracker.process(raw);

    MotorCmd cmd;
    if (g_calibrating) {
      cmd = calibrate_step(raw, dt);
    } else {
      NavInput in{line, sonar_distance_m(), g_odom.distance(), g_odom.heading(), dt};
      cmd = g_nav.update(in);
    }
    motors_set(cmd.left, cmd.right);
    publish(line, cmd, raw, micros() - t0);
  }
}
}  // namespace

void app_begin() {
  motors_begin();
  encoders_begin();
  line_sensor_begin();
  sonar_begin();

  uint16_t mn[kLineSensors], mx[kLineSensors];
  if (line_sensor_load_cal(mn, mx)) g_tracker.set_calibration(mn, mx);
  g_nav.route().parse(DEFAULT_ROUTE);

  g_queue = xQueueCreate(8, sizeof(Command));
  // control loop on core 1, wifi stack lives on core 0
  xTaskCreatePinnedToCore(control_task, "control", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
}

bool app_send(const Command& cmd) { return g_queue && xQueueSend(g_queue, &cmd, 0) == pdTRUE; }

Telemetry app_telemetry() {
  portENTER_CRITICAL(&g_mux);
  Telemetry t = g_tel;
  portEXIT_CRITICAL(&g_mux);
  return t;
}
