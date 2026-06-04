#pragma once
#include "MAX31855.h"

extern MAX31855 tc1;
extern MAX31855 tc2;

void sensors_init();
float readTop();
float readBot();
float readPcb();   // probe — wire to whichever TC you use for PCB