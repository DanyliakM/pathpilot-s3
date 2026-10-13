#include "cli.h"
#include <Arduino.h>
#include <string.h>
#include "app.h"

namespace {
bool g_log = false;

float next_float() {
  const char* s = strtok(nullptr, " ");
  return s ? atof(s) : NAN;
}

void print_status() {
  Telemetry t = app_telemetry();
  Serial.printf("state=%s route=%s step=%u junctions=%u cal=%d\n", Navigator::state_name(t.state), t.route,
                t.route_idx, t.crossings, t.calibrated);
  Serial.printf("pos=%.2f active=%u obstacle=%.2f dist=%.2f heading=%.1f loop=%luus\n", t.line.position,
                t.line.active, t.obstacle_m, t.distance_m, t.heading_deg, (unsigned long)t.loop_us);
  Serial.printf("pid=%.3f/%.3f/%.3f speed=%.2f raw=%u %u %u %u %u\n", t.kp, t.ki, t.kd, t.speed, t.raw[0], t.raw[1],
                t.raw[2], t.raw[3], t.raw[4]);
}

void execute(char* line) {
  const char* word = strtok(line, " ");
  if (!word) return;
  Command c;
  if (!strcmp(word, "start")) {
    c.type = CmdType::Start;
  } else if (!strcmp(word, "stop")) {
    c.type = CmdType::Stop;
  } else if (!strcmp(word, "cal")) {
    c.type = CmdType::Calibrate;
  } else if (!strcmp(word, "route")) {
    const char* r = strtok(nullptr, "");
    Route check;
    if (!r || !check.parse(r)) return (void)Serial.println("err: route must use L S R U F");
    c.type = CmdType::SetRoute;
    strncpy(c.text, r, sizeof(c.text) - 1);
  } else if (!strcmp(word, "pid")) {
    c.type = CmdType::SetPid;
    c.a = next_float();
    c.b = next_float();
    c.c = next_float();
    if (isnan(c.a) || isnan(c.b) || isnan(c.c)) return (void)Serial.println("err: pid <kp> <ki> <kd>");
  } else if (!strcmp(word, "speed")) {
    c.type = CmdType::SetSpeed;
    c.a = next_float();
    if (isnan(c.a)) return (void)Serial.println("err: speed <0.1..1>");
  } else if (!strcmp(word, "status")) {
    return print_status();
  } else if (!strcmp(word, "log")) {
    g_log = !g_log;
    return;
  } else {
    return (void)Serial.println("commands: start stop cal route pid speed status log");
  }
  Serial.println(app_send(c) ? "ok" : "err: queue full");
}
}  // namespace

void cli_loop() {
  static char buf[64];
  static size_t n = 0;
  while (Serial.available()) {
    char ch = char(Serial.read());
    if (ch == '\r') continue;
    if (ch != '\n') {
      if (n < sizeof(buf) - 1) buf[n++] = ch;
      continue;
    }
    buf[n] = '\0';
    n = 0;
    execute(buf);
  }
  // serial plotter friendly stream
  static uint32_t last = 0;
  if (g_log && millis() - last >= 100) {
    last = millis();
    Telemetry t = app_telemetry();
    Serial.printf("pos:%.2f ml:%.2f mr:%.2f obstacle:%.2f\n", t.line.position, t.motor_l, t.motor_r, t.obstacle_m);
  }
}
