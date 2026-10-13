#pragma once
#include <stdint.h>

void encoders_begin();
void encoders_read_delta(int32_t& left, int32_t& right);  // ticks since last call
