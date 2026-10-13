#pragma once
#include <stdint.h>

// line sensor array, analog, adc1 only (adc2 is blocked while wifi is on)
constexpr uint8_t PIN_LINE[5] = {1, 2, 3, 4, 5};
constexpr bool LINE_DARK_IS_HIGH = true;

// tb6612fng motor driver
constexpr uint8_t PIN_PWMA = 10, PIN_AIN1 = 11, PIN_AIN2 = 12;
constexpr uint8_t PIN_PWMB = 13, PIN_BIN1 = 14, PIN_BIN2 = 15;
constexpr uint8_t PIN_STBY = 16;
constexpr bool MOTOR_L_INVERT = false, MOTOR_R_INVERT = false;
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_RES_BITS = 10;
constexpr float MOTOR_DEADBAND = 0.03f;
constexpr float MOTOR_MIN_DUTY = 0.18f;

// quadrature encoders: channel a -> interrupt, channel b -> direction
constexpr uint8_t PIN_ENC_L_A = 39, PIN_ENC_L_B = 40;
constexpr uint8_t PIN_ENC_R_A = 41, PIN_ENC_R_B = 42;
constexpr bool ENC_L_INVERT = false, ENC_R_INVERT = true;
constexpr uint16_t ENC_TICKS_PER_REV = 700;
constexpr float WHEEL_DIAMETER_M = 0.043f;
constexpr float WHEEL_TRACK_M = 0.120f;

// hc-sr04, echo goes through a 5v -> 3.3v divider
constexpr uint8_t PIN_SONAR_TRIG = 6, PIN_SONAR_ECHO = 7;
constexpr uint32_t SONAR_PERIOD_MS = 60;

constexpr uint8_t PIN_BUTTON = 0;  // boot button: start / stop

constexpr uint32_t CONTROL_PERIOD_MS = 10;  // 100 hz control loop
constexpr float CAL_DURATION_S = 4.0f;
constexpr float CAL_SPEED = 0.35f;

#define WIFI_AP_SSID "PathPilot-S3"
#define WIFI_AP_PASS "pathpilot"
#define DEFAULT_ROUTE "SLRS"
