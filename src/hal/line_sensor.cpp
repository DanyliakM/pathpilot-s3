#include "line_sensor.h"
#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "line_tracker.h"

static_assert(sizeof(PIN_LINE) == kLineSensors, "PIN_LINE must match kLineSensors");

namespace {
constexpr const char* kNs = "pathpilot";
constexpr size_t kBytes = sizeof(uint16_t) * kLineSensors;
}  // namespace

void line_sensor_begin() {
  analogReadResolution(12);
  for (uint8_t pin : PIN_LINE) pinMode(pin, INPUT);
}

void line_sensor_read(uint16_t* raw) {
  for (size_t i = 0; i < kLineSensors; ++i) {
    uint16_t v = analogRead(PIN_LINE[i]);
    raw[i] = LINE_DARK_IS_HIGH ? v : uint16_t(4095 - v);
  }
}

bool line_sensor_load_cal(uint16_t* min, uint16_t* max) {
  Preferences p;
  if (!p.begin(kNs, true)) return false;
  bool ok = p.getBytes("min", min, kBytes) == kBytes && p.getBytes("max", max, kBytes) == kBytes;
  p.end();
  return ok;
}

void line_sensor_save_cal(const uint16_t* min, const uint16_t* max) {
  Preferences p;
  if (!p.begin(kNs, false)) return;
  p.putBytes("min", min, kBytes);
  p.putBytes("max", max, kBytes);
  p.end();
}
