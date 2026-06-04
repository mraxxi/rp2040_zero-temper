#include "display.h"
#include <U8g2lib.h>
#include <SPI.h>

#define GRAPH_X    0
#define GRAPH_Y   26
#define GRAPH_W  128
#define GRAPH_H   30
#define MAX_HIST  128
#define MIN_TEMP    0
#define MAX_TEMP  280

static U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, 3, 2, 1, 8);

static uint8_t histPcb[MAX_HIST];
static uint8_t histLen = 0;

static const char* PHASES[] = { "IDLE", "PREHEAT", "SOAK", "REFLOW", "COOLDOWN" };

static uint8_t tempToY(float t) {
    float norm = constrain(t, MIN_TEMP, MAX_TEMP) / (float)MAX_TEMP;
    return GRAPH_Y + GRAPH_H - (uint8_t)(norm * GRAPH_H);
}

void display_init() {
    u8g2.begin();
}

void display_draw(float top, float bot, float pcb,
                  uint32_t elapsed, uint8_t phase,
                  float setpoint, float rate,
                  uint8_t errCode, const char* errStr) {

    // push PCB history
    uint8_t val = (uint8_t)constrain(pcb, 0, 255);
    if (histLen < MAX_HIST) histPcb[histLen++] = val;
    else { memmove(histPcb, histPcb + 1, MAX_HIST - 1); histPcb[MAX_HIST - 1] = val; }

    uint8_t mm = elapsed / 60;
    uint8_t ss = elapsed % 60;
    char buf[32];

    u8g2.clearBuffer();

    // Row 1: phase + timer
    u8g2.setFont(u8g2_font_5x7_tr);
    snprintf(buf, sizeof(buf), "%-8s %02d:%02d", PHASES[phase], mm, ss);
    u8g2.drawStr(0, 7, buf);

    // Row 2: TOP + BOT
    snprintf(buf, sizeof(buf), "TOP:%3d BOT:%3d", (int)top, (int)bot);
    u8g2.drawStr(0, 16, buf);

    // PCB temp big font right side
    u8g2.setFont(u8g2_font_6x10_tr);
    snprintf(buf, sizeof(buf), "P:%3d", (int)pcb);
    uint8_t tw = u8g2.getStrWidth(buf);
    u8g2.drawStr(127 - tw, 19, buf);

    // Separator
    u8g2.drawHLine(0, GRAPH_Y - 6, 128);

    // Grid lines
    const uint8_t gridTemps[] = { 50, 100, 150, 200, 250 };
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t gy = tempToY(gridTemps[i]);
        if (gy >= GRAPH_Y && gy <= GRAPH_Y + GRAPH_H)
            for (uint8_t x = 0; x < 128; x += 6) u8g2.drawPixel(x, gy);
    }

    // Graph
    uint8_t offset = (histLen < GRAPH_W) ? (GRAPH_W - histLen) : 0;
    uint8_t count  = (histLen < GRAPH_W) ? histLen : GRAPH_W;
    for (uint8_t i = 1; i < count; i++) {
        uint8_t x = GRAPH_X + offset + i;
        u8g2.drawLine(x - 1, tempToY(histPcb[i-1]), x, tempToY(histPcb[i]));
    }

    // Bottom separator
    u8g2.drawHLine(0, GRAPH_Y + GRAPH_H - 1, 128);

    // Bottom bar
    u8g2.setFont(u8g2_font_5x7_tr);
    if (errCode) {
        snprintf(buf, sizeof(buf), "ERR %s", errStr);
    } else {
        snprintf(buf, sizeof(buf), "Sp:%3d %c%.1fC/s",
                 (int)setpoint, (rate >= 0 ? '+' : '-'), fabsf(rate));
    }
    u8g2.drawStr(0, 63, buf);

    u8g2.sendBuffer();
}