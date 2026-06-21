#include <Arduino.h>
#include <U8g2lib.h>
#include "enc_display.h"
#include "common.h"
#include "config.h"
#include "pid_control.h"
#include "tuning.h"
#include "storage.h"
#include "telemetry.h"

// ─── Display Instance (ST7920 128x64 SW-SPI) ─────────────────────────────────
static U8G2_ST7920_128X64_F_SW_SPI u8g2(
    U8G2_R0,
    PIN_DISP_CLK,
    PIN_DISP_DATA,
    PIN_DISP_CS,
    U8X8_PIN_NONE   // no reset pin
);

// ─── BGA Reflow Profiles ─────────────────────────────────────────────────────
typedef struct {
    const char *name;
    float preheat_temp;     // °C — bottom heater soak target
    float preheat_time_s;   // seconds to hold preheat
    float peak_temp;        // °C — max top zone temp
    float ramp_rate;        // °C/s — rise rate toward peak
    float soak_time_s;      // seconds at peak before cooling
    float top_delay_s;      // seconds after preheat before top heater fires
} BgaProfile;

#define NUM_PROFILES 3
static const BgaProfile k_profiles[NUM_PROFILES] = {
    { "SnPb 63/37",  150.0f, 90.0f,  183.0f, 1.5f, 45.0f, 15.0f },
    { "SAC305",      150.0f, 90.0f,  245.0f, 2.0f, 30.0f, 20.0f },
    { "Custom",      130.0f, 60.0f,  220.0f, 1.8f, 30.0f, 15.0f },
};

// ─── Preheat Config ───────────────────────────────────────────────────────────
typedef struct {
    float target_temp;    // °C
    uint16_t duration_s;  // seconds
} PreheatConfig;

static PreheatConfig g_preheat_cfg = { 150.0f, 120 };

// ─── Menu State ───────────────────────────────────────────────────────────────
static MenuState g_menu        = MENU_SPLASH;
static int8_t    g_cursor      = 0;   // current menu cursor index
static int8_t    g_scroll_top  = 0;   // top visible item in scrollable lists
static uint8_t   g_profile_sel = 0;   // selected BGA profile index
static uint32_t  g_splash_ms   = 0;   // splash entry timestamp
static uint32_t  g_run_start   = 0;   // timestamp when a run started
static uint32_t  g_display_ms  = 0;   // last display redraw

// Preheat config cursor targets
static float    *g_adj_float   = nullptr;
static uint16_t *g_adj_uint16  = nullptr;

// ─── Encoder State ────────────────────────────────────────────────────────────
static int8_t  g_enc_last_ab  = 0;
static uint8_t g_enc_state    = 0;
static EncoderEvent g_enc_event = ENC_NONE;

static uint32_t g_btn_down_ms  = 0;
static bool     g_btn_pressed  = false;

#define LONG_PRESS_MS   600UL
#define DISPLAY_RATE_MS 100UL  // max redraw rate

// ─── Display Layout Constants ─────────────────────────────────────────────────
#define LINE_H      10    // pixels per text line (small font)
#define MENU_Y0     12    // y for first menu item
#define VISIBLE_ROWS 4    // how many list items fit on screen at once
#define CURSOR_CHAR ">"

// ─── Forward declarations ─────────────────────────────────────────────────────
static void draw_screen(void);
static void draw_screen_bga(void);
static void handle_encoder(EncoderEvent ev);
static void draw_header(const char *title, bool show_temps);
static void draw_menu_list(const char * const *items, uint8_t count, int8_t cursor, int8_t scroll);
static void draw_temp_row(uint8_t y, const char *label, double temp, bool fault);
static void draw_running_bar(uint8_t y, double output_ms);
static void menu_clamp(int8_t *cursor, int8_t *scroll_top, uint8_t count);


// ─── Init ─────────────────────────────────────────────────────────────────────

void display_init(void) {
    u8g2.begin();
    u8g2.setFont(u8g2_font_5x7_tr);  // small, readable on 128x64
    u8g2.setFontPosTop();

    // Encoder pins
    pinMode(PIN_ENC_A,   INPUT_PULLUP);
    pinMode(PIN_ENC_B,   INPUT_PULLUP);
    pinMode(PIN_ENC_BTN, INPUT_PULLUP);

    g_enc_last_ab = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
    g_enc_state   = 0;

    g_menu      = MENU_SPLASH;
    g_splash_ms = millis();

    // Restore persisted preheat config and profile selection
    g_preheat_cfg.target_temp = g_settings.preheat_target_temp;
    g_preheat_cfg.duration_s  = g_settings.preheat_duration_s;
    g_profile_sel             = g_settings.profile_sel;
}

// ─── Encoder Polling ──────────────────────────────────────────────────────────
//
// Quadrature decoding with a 4-state table. Reports ENC_CW / ENC_CCW
// on each detent. Long-press detected by timing the button hold.

static const int8_t k_enc_table[16] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

EncoderEvent encoder_poll(void) {
    // Quadrature decode
    int8_t ab = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
    g_enc_state = ((g_enc_state << 2) | ab) & 0x0F;
    int8_t delta = k_enc_table[g_enc_state];

    if (delta == 1)  g_enc_event = ENC_CW;
    if (delta == -1) g_enc_event = ENC_CCW;

    // Button handling with long-press detection
    bool btn_now = (digitalRead(PIN_ENC_BTN) == LOW);
    uint32_t now = millis();

    if (btn_now && !g_btn_pressed) {
        g_btn_pressed  = true;
        g_btn_down_ms  = now;
    } else if (!btn_now && g_btn_pressed) {
        g_btn_pressed = false;
        if (now - g_btn_down_ms >= LONG_PRESS_MS) {
            g_enc_event = ENC_LONG_PRESS;
        } else {
            g_enc_event = ENC_PRESS;
        }
    }

    EncoderEvent ev = g_enc_event;
    g_enc_event = ENC_NONE;
    return ev;
}


// ─── Main Tick ────────────────────────────────────────────────────────────────

void display_tick(void) {
    EncoderEvent ev = encoder_poll();
    if (ev != ENC_NONE) {
        handle_encoder(ev);
    }

    // Throttle redraws — still responsive since encoder is polled every loop
    uint32_t now = millis();
    if (now - g_display_ms < DISPLAY_RATE_MS) return;
    g_display_ms = now;

    // Auto-advance splash after 2 seconds
    if (g_menu == MENU_SPLASH && now - g_splash_ms >= 2000UL) {
        g_menu   = MENU_MAIN;
        g_cursor = 0;
        g_scroll_top = 0;
    }

    draw_screen();
}

// ─── Drawing Helpers ──────────────────────────────────────────────────────────

static void draw_header(const char *title, bool show_temps) {
    // Title bar
    u8g2.drawStr(0, 0, title);
    u8g2.drawHLine(0, 9, 128);

    if (show_temps) {
        // Compact temp strip at very bottom
        char buf[32];
        snprintf(buf, sizeof(buf), "Z:%.0f E:%.0f",
                 g_ctx.sensors.zone_temp,
                 g_ctx.sensors.edge_temp);
        u8g2.drawStr(0, 57, buf);
    }
}

static void draw_menu_list(const char * const *items, uint8_t count,
                           int8_t cursor, int8_t scroll) {
    for (uint8_t i = 0; i < VISIBLE_ROWS && (scroll + i) < count; i++) {
        uint8_t idx = scroll + i;
        uint8_t y   = MENU_Y0 + i * LINE_H;
        if (idx == (uint8_t)cursor) {
            u8g2.drawStr(0, y, CURSOR_CHAR);
        }
        u8g2.drawStr(8, y, items[idx]);
    }
    // Scroll indicator if list overflows
    if ((int)count > VISIBLE_ROWS) {
        uint8_t bar_h = (VISIBLE_ROWS * LINE_H * VISIBLE_ROWS) / count;
        uint8_t bar_y = MENU_Y0 + (scroll * VISIBLE_ROWS * LINE_H) / count;
        u8g2.drawBox(126, bar_y, 2, bar_h);
    }
}

static void draw_temp_row(uint8_t y, const char *label, double temp, bool fault) {
    char buf[24];
    if (fault) {
        snprintf(buf, sizeof(buf), "%s: FAULT", label);
    } else {
        snprintf(buf, sizeof(buf), "%s: %.1f C", label, temp);
    }
    u8g2.drawStr(0, y, buf);
}

static void draw_running_bar(uint8_t y, double output_ms) {
    // Show PID output as a small progress bar
    uint8_t w = (uint8_t)((output_ms / SSR_WINDOW_MS) * 60.0);
    u8g2.drawFrame(65, y, 62, 7);
    if (w > 0) u8g2.drawBox(65, y, w, 7);
}

static void menu_clamp(int8_t *cursor, int8_t *scroll_top, uint8_t count) {
    if (*cursor < 0)              *cursor = 0;
    if (*cursor >= (int8_t)count) *cursor = count - 1;
    if (*cursor < *scroll_top)                        *scroll_top = *cursor;
    if (*cursor >= *scroll_top + VISIBLE_ROWS)        *scroll_top = *cursor - VISIBLE_ROWS + 1;
    if (*scroll_top < 0)                              *scroll_top = 0;
}


// ─── Screen Renderer ──────────────────────────────────────────────────────────

static void draw_screen(void) {
    u8g2.clearBuffer();

    switch (g_menu) {

    // ── Splash ──────────────────────────────────────────────────────────────
    case MENU_SPLASH: {
        u8g2.setFont(u8g2_font_8x13B_tr);
        u8g2.drawStr(20, 16, "temper2040");
        u8g2.setFont(u8g2_font_5x7_tr);
        u8g2.drawStr(30, 34, "BGA rework station");
        u8g2.drawStr(40, 46, "initialising...");
        break;
    }

    // ── Main Menu ───────────────────────────────────────────────────────────
    case MENU_MAIN: {
        static const char * const items[] = {
            "Preheat",
            "BGA Rework",
            "Autotune",
            "Configs",
        };
        draw_header("[ MAIN MENU ]", true);
        draw_menu_list(items, 4, g_cursor, g_scroll_top);
        break;
    }

    // ── Preheat Menu ────────────────────────────────────────────────────────
    case MENU_PREHEAT: {
        static const char * const items[] = {
            "Configure",
            "Start Preheat",
            "< Back",
        };
        draw_header("Preheat", true);
        draw_menu_list(items, 3, g_cursor, g_scroll_top);
        break;
    }

    // ── Preheat Config ──────────────────────────────────────────────────────
    case MENU_PREHEAT_CONFIG: {
        char buf[32];
        draw_header("Preheat Config", false);
        snprintf(buf, sizeof(buf), "%sTarget: %.0f C",
                 (g_cursor == 0 ? CURSOR_CHAR : " "), (double)g_preheat_cfg.target_temp);
        u8g2.drawStr(0, MENU_Y0, buf);
        snprintf(buf, sizeof(buf), "%sDuration: %ds",
                 (g_cursor == 1 ? CURSOR_CHAR : " "), g_preheat_cfg.duration_s);
        u8g2.drawStr(0, MENU_Y0 + LINE_H, buf);
        u8g2.drawStr(0, MENU_Y0 + 2 * LINE_H, "  [Press] confirm");
        u8g2.drawStr(0, MENU_Y0 + 3 * LINE_H, "  [Hold]  back");
        break;
    }

    // ── Preheat Running ─────────────────────────────────────────────────────
    case MENU_PREHEAT_RUNNING: {
        char buf[32];
        uint32_t elapsed = (millis() - g_run_start) / 1000UL;
        draw_header("PREHEAT", false);
        snprintf(buf, sizeof(buf), "Set: %.0f  Now: %.1f C",
                 (double)g_preheat_cfg.target_temp, g_ctx.sensors.edge_temp);
        u8g2.drawStr(0, MENU_Y0, buf);
        snprintf(buf, sizeof(buf), "Elapsed: %lus / %ds",
                 (unsigned long)elapsed, g_preheat_cfg.duration_s);
        u8g2.drawStr(0, MENU_Y0 + LINE_H, buf);
        // PID output bar
        u8g2.drawStr(0, MENU_Y0 + 2 * LINE_H, "Out:");
        draw_running_bar(MENU_Y0 + 2 * LINE_H, g_ctx.pid_bot.output);
        // Fault indicator
        if (g_ctx.sensors.edge_fault)
            u8g2.drawStr(0, MENU_Y0 + 3 * LINE_H, "!! EDGE TC FAULT !!");
        else
            u8g2.drawStr(0, MENU_Y0 + 3 * LINE_H, "[Hold] abort");
        break;
    }

    default:
        draw_screen_bga();  // handles BGA, autotune, configs, error screens
        return;
    }

    u8g2.sendBuffer();
}

static void draw_screen_bga(void) {
    u8g2.clearBuffer();

    switch (g_menu) {

    // ── BGA Menu ────────────────────────────────────────────────────────────
    case MENU_BGA: {
        static const char * const items[] = {
            "Configure",
            "Select Profile",
            "Start BGA Rework",
            "< Back",
        };
        draw_header("BGA Rework", true);
        draw_menu_list(items, 4, g_cursor, g_scroll_top);
        break;
    }

    // ── BGA Profile Select ──────────────────────────────────────────────────
    case MENU_BGA_PROFILE_SELECT: {
        static const char *names[NUM_PROFILES];
        for (uint8_t i = 0; i < NUM_PROFILES; i++) names[i] = k_profiles[i].name;
        draw_header("Select Profile", false);
        draw_menu_list(names, NUM_PROFILES, g_cursor, g_scroll_top);
        // Show selected marker
        char buf[24];
        snprintf(buf, sizeof(buf), "Active: %s", k_profiles[g_profile_sel].name);
        u8g2.drawStr(0, 57, buf);
        break;
    }

    // ── BGA Config ──────────────────────────────────────────────────────────
    case MENU_BGA_CONFIG: {
        const BgaProfile *p = &k_profiles[g_profile_sel];
        char buf[32];
        draw_header("BGA Config", false);
        snprintf(buf, sizeof(buf), "Profile: %s", p->name);
        u8g2.drawStr(0, MENU_Y0, buf);
        snprintf(buf, sizeof(buf), "Peak: %.0f C  Soak: %.0fs", (double)p->peak_temp, (double)p->soak_time_s);
        u8g2.drawStr(0, MENU_Y0 + LINE_H, buf);
        snprintf(buf, sizeof(buf), "Ramp: %.1f C/s", (double)p->ramp_rate);
        u8g2.drawStr(0, MENU_Y0 + 2 * LINE_H, buf);
        u8g2.drawStr(0, MENU_Y0 + 3 * LINE_H, "[Press] OK  [Hold] back");
        break;
    }

    // ── BGA Running ─────────────────────────────────────────────────────────
    case MENU_BGA_RUNNING: {
        char buf[32];
        uint32_t elapsed = (millis() - g_run_start) / 1000UL;
        draw_header("BGA RUNNING", false);
        snprintf(buf, sizeof(buf), "Zone: %.1f  Edge: %.1f",
                 g_ctx.sensors.zone_temp, g_ctx.sensors.edge_temp);
        u8g2.drawStr(0, MENU_Y0, buf);
        snprintf(buf, sizeof(buf), "Set T: %.0f  B: %.0f",
                 g_ctx.pid_top.setpoint, g_ctx.pid_bot.setpoint);
        u8g2.drawStr(0, MENU_Y0 + LINE_H, buf);
        // Dual PID output bars
        u8g2.drawStr(0,  MENU_Y0 + 2 * LINE_H, "T:");
        draw_running_bar(MENU_Y0 + 2 * LINE_H, g_ctx.pid_top.output);
        u8g2.drawStr(0,  MENU_Y0 + 3 * LINE_H, "B:");
        draw_running_bar(MENU_Y0 + 3 * LINE_H, g_ctx.pid_bot.output);
        // Elapsed time
        snprintf(buf, sizeof(buf), "t=%lus", (unsigned long)elapsed);
        u8g2.drawStr(90, 0, buf);
        if (g_ctx.state == STATE_ABORT || g_ctx.state == STATE_ERROR)
            u8g2.drawStr(0, 57, "!! ABORTED !!");
        else
            u8g2.drawStr(0, 57, "[Hold] abort");
        break;
    }

    // ── Autotune Select ──────────────────────────────────────────────────────
    case MENU_TUNE_SELECT: {
        static const char * const items[] = {
            "Tune Top PID",
            "Tune Bottom PID",
            "< Back",
        };
        draw_header("Autotune", true);
        draw_menu_list(items, 3, g_cursor, g_scroll_top);
        break;
    }

    // ── Autotune Running ─────────────────────────────────────────────────────
    case MENU_TUNE_RUNNING: {
        char buf[32];
        draw_header("TUNING...", false);
        const char *phase_str = "Settling";
        if (g_tune.phase == TUNE_RELAY) phase_str = "Relay";
        if (g_tune.phase == TUNE_DONE)  phase_str = "Done!";
        snprintf(buf, sizeof(buf), "Phase: %s", phase_str);
        u8g2.drawStr(0, MENU_Y0, buf);
        snprintf(buf, sizeof(buf), "Target: %.0f C", g_tune.tune_setpoint);
        u8g2.drawStr(0, MENU_Y0 + LINE_H, buf);
        snprintf(buf, sizeof(buf), "Cycles: %d / %d",
                 g_tune.amplitude_count, g_tune.cycles_needed);
        u8g2.drawStr(0, MENU_Y0 + 2 * LINE_H, buf);
        u8g2.drawStr(0, MENU_Y0 + 3 * LINE_H, "[Hold] abort");
        break;
    }

    // ── Autotune Result ──────────────────────────────────────────────────────
    case MENU_TUNE_RESULT: {
        char buf[32];
        draw_header("Tune Result", false);
        snprintf(buf, sizeof(buf), "Kp: %.3f", g_tune.result_kp);
        u8g2.drawStr(0, MENU_Y0, buf);
        snprintf(buf, sizeof(buf), "Ki: %.4f", g_tune.result_ki);
        u8g2.drawStr(0, MENU_Y0 + LINE_H, buf);
        snprintf(buf, sizeof(buf), "Kd: %.3f", g_tune.result_kd);
        u8g2.drawStr(0, MENU_Y0 + 2 * LINE_H, buf);
        u8g2.drawStr(0, MENU_Y0 + 3 * LINE_H, "[Press] apply  [Hold] discard");
        break;
    }

    // ── Configs Menu ─────────────────────────────────────────────────────────
    case MENU_CONFIGS: {
        static const char * const items[] = {
            "Beeper: ON",    // TODO: toggle dynamically
            "Heater logging",
            "< Back",
        };
        draw_header("Configs", true);
        draw_menu_list(items, 3, g_cursor, g_scroll_top);
        break;
    }

    // ── Error Screen ────────────────────────────────────────────────────────
    case MENU_ERROR: {
        u8g2.drawStr(20, 10, "!! ERROR !!");
        if (g_ctx.sensors.zone_fault)
            u8g2.drawStr(0, 24, "Zone TC fault");
        if (g_ctx.sensors.edge_fault)
            u8g2.drawStr(0, 34, "Edge TC fault");
        u8g2.drawStr(0, 50, "[Press] return to menu");
        break;
    }

    default:
        break;
    }

    u8g2.sendBuffer();
}


// ─── Encoder Event Handler ────────────────────────────────────────────────────
//
// CW/CCW navigate lists or adjust values.
// PRESS   = select / confirm.
// LONG_PRESS = back / abort (universal).

static void handle_encoder(EncoderEvent ev) {
    int8_t sel = g_cursor; // capture before any nav change

    switch (g_menu) {

    // ── Main Menu ────────────────────────────────────────────────────────────
    case MENU_MAIN: {
        const uint8_t COUNT = 4;
        if (ev == ENC_CW)  { g_cursor++; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        if (ev == ENC_CCW) { g_cursor--; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        if (ev == ENC_PRESS) {
            g_cursor = 0; g_scroll_top = 0;
            switch (sel) {
                case 0: g_menu = MENU_PREHEAT;      break;
                case 1: g_menu = MENU_BGA;          break;
                case 2: g_menu = MENU_TUNE_SELECT;  break;
                case 3: g_menu = MENU_CONFIGS;      break;
                default: break;
            }
        }
        break;
    }

    // ── Preheat Menu ─────────────────────────────────────────────────────────
    case MENU_PREHEAT: {
        const uint8_t COUNT = 3;
        if (ev == ENC_CW)  { g_cursor++; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        if (ev == ENC_CCW) { g_cursor--; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        if (ev == ENC_PRESS) {
            g_cursor = 0; g_scroll_top = 0;
            switch (sel) {
                case 0: g_menu = MENU_PREHEAT_CONFIG; break;
                case 1:
                    // Start preheat
                    pid_set_setpoint(&g_ctx.pid_bot, g_preheat_cfg.target_temp);
                    g_ctx.state   = STATE_PREHEAT;
                    g_run_start   = millis();
                    g_menu        = MENU_PREHEAT_RUNNING;
                    break;
                case 2:
                    g_menu = MENU_MAIN;
                    break;
                default: break;
            }
        }
        if (ev == ENC_LONG_PRESS) { g_cursor = 0; g_scroll_top = 0; g_menu = MENU_MAIN; }
        break;
    }

    // ── Preheat Config ────────────────────────────────────────────────────────
    case MENU_PREHEAT_CONFIG: {
        // cursor 0 = target temp, cursor 1 = duration
        const uint8_t COUNT = 2;
        if (ev == ENC_CW)  { g_cursor++; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        if (ev == ENC_CCW) { g_cursor--; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        // Value adjustment uses 'sel' (cursor before any nav change) so the
        // item being edited matches what was highlighted before the turn.
        if (ev == ENC_CW || ev == ENC_CCW) {
            float delta = (ev == ENC_CW) ? 5.0f : -5.0f;
            if (sel == 0) {
                g_preheat_cfg.target_temp += delta;
                if (g_preheat_cfg.target_temp < 50.0f)  g_preheat_cfg.target_temp = 50.0f;
                if (g_preheat_cfg.target_temp > 300.0f) g_preheat_cfg.target_temp = 300.0f;
            } else {
                g_preheat_cfg.duration_s = (uint16_t)((int)g_preheat_cfg.duration_s + (int)(delta));
                if (g_preheat_cfg.duration_s < 10)  g_preheat_cfg.duration_s = 10;
                if (g_preheat_cfg.duration_s > 600) g_preheat_cfg.duration_s = 600;
            }
        }
        if (ev == ENC_PRESS || ev == ENC_LONG_PRESS) {
            // Persist updated preheat config
            g_settings.preheat_target_temp = g_preheat_cfg.target_temp;
            g_settings.preheat_duration_s  = g_preheat_cfg.duration_s;
            storage_save();
            g_cursor = 0; g_scroll_top = 0; g_menu = MENU_PREHEAT;
        }
        break;
    }

    // ── Preheat Running ───────────────────────────────────────────────────────
    case MENU_PREHEAT_RUNNING: {
        if (ev == ENC_LONG_PRESS) {
            g_ctx.state = STATE_IDLE;
            g_cursor = 0; g_scroll_top = 0;
            g_menu = MENU_PREHEAT;
        }
        // Auto-transition to complete when duration elapses
        if ((millis() - g_run_start) / 1000UL >= g_preheat_cfg.duration_s) {
            g_ctx.state = STATE_IDLE;
            g_cursor = 0; g_scroll_top = 0;
            g_menu = MENU_PREHEAT;
        }
        break;
    }

    // ── BGA Menu ──────────────────────────────────────────────────────────────
    case MENU_BGA: {
        const uint8_t COUNT = 4;
        if (ev == ENC_CW)  { g_cursor++; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        if (ev == ENC_CCW) { g_cursor--; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        if (ev == ENC_PRESS) {
            g_cursor = 0; g_scroll_top = 0;
            switch (sel) {
                case 0: g_menu = MENU_BGA_CONFIG;         break;
                case 1: g_menu = MENU_BGA_PROFILE_SELECT; break;
                case 2:
                    // Start BGA rework using selected profile
                    pid_set_setpoint(&g_ctx.pid_top,
                                     k_profiles[g_profile_sel].peak_temp);
                    pid_set_setpoint(&g_ctx.pid_bot,
                                     k_profiles[g_profile_sel].preheat_temp);
                    g_ctx.state = STATE_BGA_REWORK;
                    g_run_start = millis();
                    g_menu      = MENU_BGA_RUNNING;
                    break;
                case 3: g_menu = MENU_MAIN; break;
                default: break;
            }
        }
        if (ev == ENC_LONG_PRESS) { g_cursor = 0; g_scroll_top = 0; g_menu = MENU_MAIN; }
        break;
    }

    // ── BGA Profile Select ────────────────────────────────────────────────────
    case MENU_BGA_PROFILE_SELECT: {
        if (ev == ENC_CW)  { g_cursor++; menu_clamp(&g_cursor, &g_scroll_top, NUM_PROFILES); }
        if (ev == ENC_CCW) { g_cursor--; menu_clamp(&g_cursor, &g_scroll_top, NUM_PROFILES); }
        if (ev == ENC_PRESS) {
            g_profile_sel = (uint8_t)g_cursor;
            g_settings.profile_sel = g_profile_sel;
            storage_save();
            g_cursor = 0; g_scroll_top = 0;
            g_menu = MENU_BGA;
        }
        if (ev == ENC_LONG_PRESS) { g_cursor = 0; g_scroll_top = 0; g_menu = MENU_BGA; }
        break;
    }

    // ── BGA Config (read-only view) ───────────────────────────────────────────
    case MENU_BGA_CONFIG: {
        if (ev == ENC_PRESS || ev == ENC_LONG_PRESS) {
            g_cursor = 0; g_scroll_top = 0; g_menu = MENU_BGA;
        }
        break;
    }

    // ── BGA Running ───────────────────────────────────────────────────────────
    case MENU_BGA_RUNNING: {
        if (ev == ENC_LONG_PRESS) {
            g_ctx.state = STATE_ABORT;
            g_cursor = 0; g_scroll_top = 0;
            g_menu = MENU_BGA;
        }
        // Transition to cooling on completion
        if (g_ctx.state == STATE_COOLING || g_ctx.state == STATE_COMPLETE) {
            g_cursor = 0; g_scroll_top = 0;
            g_menu = MENU_MAIN;
        }
        if (g_ctx.state == STATE_ABORT || g_ctx.state == STATE_ERROR) {
            // Stay on running screen showing ABORTED, press to dismiss
            if (ev == ENC_PRESS) {
                g_cursor = 0; g_scroll_top = 0;
                g_menu = MENU_BGA;
            }
        }
        break;
    }

    // ── Autotune Select ───────────────────────────────────────────────────────
    case MENU_TUNE_SELECT: {
        const uint8_t COUNT = 3;
        if (ev == ENC_CW)  { g_cursor++; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        if (ev == ENC_CCW) { g_cursor--; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        if (ev == ENC_PRESS) {
            g_cursor = 0; g_scroll_top = 0;
            switch (sel) {
                case 0:
                    tuning_start(150.0, true);
                    g_menu = MENU_TUNE_RUNNING;
                    break;
                case 1:
                    tuning_start(150.0, false);
                    g_menu = MENU_TUNE_RUNNING;
                    break;
                case 2: g_menu = MENU_MAIN; break;
                default: break;
            }
        }
        if (ev == ENC_LONG_PRESS) { g_cursor = 0; g_scroll_top = 0; g_menu = MENU_MAIN; }
        break;
    }

    // ── Autotune Running ──────────────────────────────────────────────────────
    case MENU_TUNE_RUNNING: {
        if (ev == ENC_LONG_PRESS) {
            tuning_abort();
            g_cursor = 0; g_scroll_top = 0;
            g_menu = MENU_TUNE_SELECT;
        }
        // Auto-advance when tune completes
        if (tuning_is_done()) {
            g_cursor = 0; g_scroll_top = 0;
            g_menu = MENU_TUNE_RESULT;
        }
        break;
    }

    // ── Autotune Result ───────────────────────────────────────────────────────
    case MENU_TUNE_RESULT: {
        if (ev == ENC_PRESS) {
            tuning_apply_results();
            g_cursor = 0; g_scroll_top = 0;
            g_menu = MENU_MAIN;
        }
        if (ev == ENC_LONG_PRESS) {
            // Discard — don't apply
            g_cursor = 0; g_scroll_top = 0;
            g_menu = MENU_MAIN;
        }
        break;
    }

    // ── Configs Menu ──────────────────────────────────────────────────────────
    case MENU_CONFIGS: {
        const uint8_t COUNT = 3;
        if (ev == ENC_CW)  { g_cursor++; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        if (ev == ENC_CCW) { g_cursor--; menu_clamp(&g_cursor, &g_scroll_top, COUNT); }
        if (ev == ENC_PRESS) {
            switch (sel) {
                case 2: g_cursor = 0; g_scroll_top = 0; g_menu = MENU_MAIN; break;
                default: break; // TODO: toggle beeper, heater logging
            }
        }
        if (ev == ENC_LONG_PRESS) { g_cursor = 0; g_scroll_top = 0; g_menu = MENU_MAIN; }
        break;
    }

    // ── Error Screen ──────────────────────────────────────────────────────────
    case MENU_ERROR: {
        if (ev == ENC_PRESS) {
            g_ctx.state = STATE_IDLE;
            g_cursor = 0; g_scroll_top = 0;
            g_menu = MENU_MAIN;
        }
        break;
    }

    default:
        break;
    }

    // Global error escalation: if TC faults during a run, show error screen
    if ((g_ctx.state == STATE_ABORT || g_ctx.state == STATE_ERROR) &&
        g_menu != MENU_BGA_RUNNING &&
        g_menu != MENU_PREHEAT_RUNNING &&
        g_menu != MENU_TUNE_RUNNING &&
        g_menu != MENU_ERROR) {
        g_menu = MENU_ERROR;
    }
}

