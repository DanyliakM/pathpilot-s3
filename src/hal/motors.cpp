#include "motors.h"
#include <Arduino.h>
#include "config.h"

namespace {
struct Channel {
  uint8_t pwm, in1, in2, ledc;
  bool invert;
};

const Channel kLeft{PIN_PWMA, PIN_AIN1, PIN_AIN2, 0, MOTOR_L_INVERT};
const Channel kRight{PIN_PWMB, PIN_BIN1, PIN_BIN2, 1, MOTOR_R_INVERT};
constexpr uint32_t kMaxDuty = (1u << PWM_RES_BITS) - 1;

void pwm_attach(const Channel& c) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(c.pwm, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(c.ledc, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(c.pwm, c.ledc);
#endif
}

void pwm_write(const Channel& c, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(c.pwm, duty);
#else
  ledcWrite(c.ledc, duty);
#endif
}

void apply(const Channel& c, float v) {
  if (c.invert) v = -v;
  v = constrain(v, -1.0f, 1.0f);
  float mag = fabsf(v);
  if (mag < MOTOR_DEADBAND) {
    digitalWrite(c.in1, LOW);
    digitalWrite(c.in2, LOW);
    pwm_write(c, 0);
    return;
  }
  // compensate static friction of small gear motors
  mag = MOTOR_MIN_DUTY + mag * (1.0f - MOTOR_MIN_DUTY);
  digitalWrite(c.in1, v > 0 ? HIGH : LOW);
  digitalWrite(c.in2, v < 0 ? HIGH : LOW);
  pwm_write(c, uint32_t(mag * kMaxDuty));
}
}  // namespace

void motors_begin() {
  for (const Channel* c : {&kLeft, &kRight}) {
    pinMode(c->in1, OUTPUT);
    pinMode(c->in2, OUTPUT);
    pwm_attach(*c);
  }
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);
  motors_stop();
}

void motors_set(float left, float right) {
  apply(kLeft, left);
  apply(kRight, right);
}

void motors_stop() { motors_set(0.0f, 0.0f); }
