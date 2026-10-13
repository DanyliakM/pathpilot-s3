#pragma once

void sonar_begin();
void sonar_trigger();       // non-blocking, echo is measured in an interrupt
float sonar_distance_m();   // -1 when nothing is in range
