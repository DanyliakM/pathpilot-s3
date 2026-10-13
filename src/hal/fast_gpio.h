#pragma once
#include <esp_attr.h>
#include <soc/gpio_reg.h>
#include <stdint.h>

// direct register read: safe inside iram isrs while flash cache is disabled (nvs writes)
static inline bool IRAM_ATTR fast_read(uint8_t pin) {
  return pin < 32 ? (REG_READ(GPIO_IN_REG) >> pin) & 1u : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1u;
}
