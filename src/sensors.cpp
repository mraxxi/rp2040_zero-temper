#include "sensors.h"

#define PIN_MISO  29
#define PIN_CLK   28
#define PIN_CS1   12
#define PIN_CS2   13

MAX31855 tc1(PIN_CLK, PIN_CS1, PIN_MISO);
MAX31855 tc2(PIN_CLK, PIN_CS2, PIN_MISO);

void sensors_init() {
    tc1.begin();
    tc2.begin();
}

float readTop() {
    double v = tc1.getTemperature();
    return isnan(v) ? -999.0f : (float)v;
}

float readBot() {
    double v = tc2.getTemperature();
    return isnan(v) ? -999.0f : (float)v;
}

float readPcb() {
    // swap to tc2 if your probe is on CS2
    return readTop();
}