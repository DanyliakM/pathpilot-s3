#pragma once
#include <stdint.h>

void line_sensor_begin();
void line_sensor_read(uint16_t* raw);  // kLineSensors values, higher = darker
bool line_sensor_load_cal(uint16_t* min, uint16_t* max);
void line_sensor_save_cal(const uint16_t* min, const uint16_t* max);
