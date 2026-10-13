#pragma once

void motors_begin();
void motors_set(float left, float right);  // -1..1, positive = forward
void motors_stop();
