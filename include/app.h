#pragma once
#include <stdint.h>
#include "line_tracker.h"
#include "navigator.h"

enum class CmdType : uint8_t { Start, Stop, Calibrate, SetRoute, SetPid, SetSpeed };

struct Command {
  CmdType type = CmdType::Stop;
  float a = 0.0f, b = 0.0f, c = 0.0f;
  char text[33] = {};
};

struct Telemetry {
  NavState state = NavState::Idle;
  bool running = false, calibrating = false, calibrated = false;
  LineReading line;
  uint16_t raw[kLineSensors] = {};
  float obstacle_m = -1.0f, distance_m = 0.0f, heading_deg = 0.0f, x_m = 0.0f, y_m = 0.0f;
  float motor_l = 0.0f, motor_r = 0.0f;
  float kp = 0.0f, ki = 0.0f, kd = 0.0f, speed = 0.0f;
  uint16_t crossings = 0;
  uint8_t route_idx = 0;
  char route[33] = {};
  uint32_t loop_us = 0;
};

void app_begin();
bool app_send(const Command& cmd);  // thread-safe, consumed by the control task
Telemetry app_telemetry();          // thread-safe snapshot
