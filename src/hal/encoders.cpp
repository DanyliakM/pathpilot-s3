#include "encoders.h"
#include <Arduino.h>
#include "config.h"
#include "fast_gpio.h"

namespace {
struct Encoder {
  uint8_t pin_a, pin_b;
  bool invert;
  volatile int32_t ticks;
  int32_t last;
};

Encoder g_left{PIN_ENC_L_A, PIN_ENC_L_B, ENC_L_INVERT, 0, 0};
Encoder g_right{PIN_ENC_R_A, PIN_ENC_R_B, ENC_R_INVERT, 0, 0};

void IRAM_ATTR on_edge(void* arg) {
  auto* e = static_cast<Encoder*>(arg);
  bool b = fast_read(e->pin_b);
  e->ticks += (b != e->invert) ? -1 : 1;
}

int32_t delta(Encoder& e) {
  int32_t now = e.ticks;  // 32-bit aligned read is atomic on xtensa
  int32_t d = now - e.last;
  e.last = now;
  return d;
}
}  // namespace

void encoders_begin() {
  for (Encoder* e : {&g_left, &g_right}) {
    pinMode(e->pin_a, INPUT_PULLUP);
    pinMode(e->pin_b, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(e->pin_a), on_edge, e, RISING);
  }
}

void encoders_read_delta(int32_t& left, int32_t& right) {
  left = delta(g_left);
  right = delta(g_right);
}
