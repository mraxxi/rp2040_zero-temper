#pragma once
#include <stdint.h>
#include "common.h"

// ─── Init ─────────────────────────────────────────────────────────────────────
void sensors_init(void);
void ssr_init(void);

// ─── Sensor Reads ─────────────────────────────────────────────────────────────
// Returns true if both mission-critical TCs (zone + edge) are healthy.
bool sensors_read_all(void);

double read_max31855(uint8_t cs_pin);
double read_max6675(uint8_t cs_pin);

// ─── SSR Control ──────────────────────────────────────────────────────────────
void ssr_set_output(SSRChannel *ch, double output_ms);
void ssr_tick(SSRChannel *ch, uint8_t pin);
void ssr_all_off(void);
