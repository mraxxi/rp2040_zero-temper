#pragma once
#include "common.h"

// ─── Init ─────────────────────────────────────────────────────────────────────
void display_init(void);

// ─── Main Tick ────────────────────────────────────────────────────────────────
// Call every loop iteration. Handles encoder polling, menu navigation,
// and partial display updates.
void display_tick(void);

// ─── Encoder ──────────────────────────────────────────────────────────────────
// Poll the rotary encoder and return the current event.
// Clears the event after reading (one event per call).
EncoderEvent encoder_poll(void);
