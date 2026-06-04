#pragma once
#include <Arduino.h>

void display_init();
void display_draw(float top, float bot, float pcb,
                  uint32_t elapsed, uint8_t phase,
                  float setpoint, float rate,
                  uint8_t errCode, const char* errStr);