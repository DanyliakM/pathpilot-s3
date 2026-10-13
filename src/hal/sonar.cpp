#include "sonar.h"
#include <Arduino.h>
#include "config.h"
#include "fast_gpio.h"

namespace {
constexpr float kMaxRangeM = 1.5f;
constexpr uint32_t kStaleMs = 250;

volatile uint32_t g_rise_us = 0;
volatile uint32_t g_width_us = 0;
volatile uint32_t g_echo_ms = 0;

void IRAM_ATTR on_echo() {
  uint32_t now = micros();
  if (fast_read(PIN_SONAR_ECHO)) {
    g_rise_us = now;
  } else {
    g_width_us = now - g_rise_us;
    g_echo_ms = millis();
  }
}
}  // namespace

void sonar_begin() {
  pinMode(PIN_SONAR_TRIG, OUTPUT);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  pinMode(PIN_SONAR_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SONAR_ECHO), on_echo, CHANGE);
}

void sonar_trigger() {
  digitalWrite(PIN_SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);
}

float sonar_distance_m() {
  if (millis() - g_echo_ms > kStaleMs) return -1.0f;
  float d = float(g_width_us) * 343.0f / 2.0f / 1e6f;
  return (d > 0.02f && d < kMaxRangeM) ? d : -1.0f;
}
