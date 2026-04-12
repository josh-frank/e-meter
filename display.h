/**
 * display.h
 *
 * Display modes for e-meter.ino.
 * Cycle with the XIAO built-in button (GPIO0 / D0).
 *
 * Modes:
 *   0  DISP_POLYGRAM  — full-screen 30-second rolling waveform
 *   1  DISP_NUMBERS   — µS + delta + clock + status icons (original layout)
 *   2  DISP_BLANK     — placeholder (Detecto drums, coming soon)
 *
 * Required changes in e-meter.ino:
 *   - PolyPoint struct removed (defined here)
 *   - g_poly, g_poly_head, g_poly_len have no 'static'
 *   - OLED_W/H, POLY_MAX, WINDOW_MS, TZ_OFFSET constants removed
 *   - draw_polygram() and renderDisplay() removed (or stay commented out)
 *   - #include "display.h" after PCF8563.h
 *   - display_setup() after u8g2.begin() in setup()
 *   - display_render(...) in loop() replaces renderDisplay()
 */

#pragma once
#include <Arduino.h>
#include <U8g2lib.h>

// ── Constants ─────────────────────────────────────────────────────────────────
#define OLED_W    128
#define OLED_H    64
#define POLY_MAX  600
#define WINDOW_MS 30000UL
#define TZ_OFFSET 0

// ── PolyPoint ─────────────────────────────────────────────────────────────────
struct PolyPoint { uint32_t t_ms; float uS; };

// ── Externs — defined in e-meter.ino (must NOT be static there) ──────────────
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
extern PolyPoint g_poly[];
extern int       g_poly_head;
extern int       g_poly_len;

// Transport state externs — used by status icons in render_numbers
// These are conditionally defined in e-meter.ino; guard with the same ifdefs.
#ifdef USE_SD
  extern bool g_recording;
#endif
#ifdef USE_BLE
  extern bool g_bleConnected;
#endif

// DateTime forward declaration — full definition comes from PCF8563.h
struct DateTime;

// ── Button ───────────────────────────────────────────────────────────────────
static const uint8_t  BTN_PIN      = D1;   // XIAO Expansion Board button
static const uint32_t BTN_DEBOUNCE = 50;  // ms

// ── Mode ─────────────────────────────────────────────────────────────────────
#define DISP_POLYGRAM  0
#define DISP_NUMBERS   1
#define DISP_BLANK     2
#define DISP_MODE_MAX  3

volatile uint8_t g_display_mode = DISP_POLYGRAM;
volatile uint32_t g_btn_last_ms = 0;  // volatile — written in ISR

// ── ISR ──────────────────────────────────────────────────────────────────────
void IRAM_ATTR btn_isr() {
  uint32_t now = millis();
  if (now - g_btn_last_ms > BTN_DEBOUNCE) {
    g_display_mode = (g_display_mode + 1) % DISP_MODE_MAX;
    g_btn_last_ms  = now;
  }
}

// ── Setup — call once after u8g2.begin() ─────────────────────────────────────
void display_setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Mode 0 — POLYGRAM
//  Full-screen 30-second rolling waveform, no labels.
// ─────────────────────────────────────────────────────────────────────────────
static void render_polygram() {
  if (g_poly_len < 2) return;

  uint32_t now = millis();

  float lo = g_poly[g_poly_head].uS, hi = lo;
  for (int i = 1; i < g_poly_len; i++) {
    float v = g_poly[(g_poly_head + i) % POLY_MAX].uS;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  float pad   = max((hi - lo) * 0.2f, 2.0f);
  float vmin  = lo - pad;
  float range = (hi + pad) - (lo - pad);
  if (range < 1.0f) range = 1.0f;

  int prev_px = -1, prev_py = -1;
  for (int i = 0; i < g_poly_len; i++) {
    PolyPoint& p = g_poly[(g_poly_head + i) % POLY_MAX];

    float age_ratio = (float)(now - p.t_ms) / (float)WINDOW_MS;
    int px = constrain((int)(OLED_W * (1.0f - age_ratio)), 0, OLED_W - 1);

    float norm_y = 1.0f - (p.uS - vmin) / range;
    int py = constrain((int)(norm_y * (OLED_H - 1)), 0, OLED_H - 1);

    if (prev_px >= 0) u8g2.drawLine(prev_px, prev_py, px, py);
    prev_px = px;
    prev_py = py;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Mode 1 — NUMBERS
//  Original renderDisplay() layout:
//    y=0..13   µS reading (large)
//    y=14..23  clock (left) + transport status icons (right)
//    y=24..32  delta + compressed delta
//    y=33..43  divider + polygram strip
// ─────────────────────────────────────────────────────────────────────────────
static void render_numbers(float uS, float delta, float delta_c,
                            const DateTime& dt) {
  // Row 1 — µS value
  u8g2.setFont(u8g2_font_7x14B_tf);
  char us_str[20];
  snprintf(us_str, sizeof(us_str), "%.2f uS", uS);
  u8g2.drawUTF8(0, 13, us_str);

  // Row 2 — clock left, status icons right
  u8g2.setFont(u8g2_font_5x7_tr);
  char time_str[12];
  int display_hour = (dt.hour + TZ_OFFSET + 24) % 24;
  snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
           display_hour, dt.minute, dt.second);
  u8g2.drawStr(0, 23, time_str);

  // Build status string (W=wifi, B=ble, */-=sd)
  char status[8] = "";
  int  scol = 0;
  #ifdef USE_SD
    status[scol++] = g_recording ? '*' : '-';
  #endif
  #ifdef USE_BLE
    status[scol++] = g_bleConnected ? 'B' : 'b';
  #endif
  #ifdef USE_WIFI
    status[scol++] = (WiFi.status() == WL_CONNECTED) ? 'W' : 'w';
  #endif
  status[scol] = '\0';
  // Reverse so order reads W B * left-to-right
  for (int i = 0, j = scol - 1; i < j; i++, j--) {
    char tmp = status[i]; status[i] = status[j]; status[j] = tmp;
  }
  u8g2.drawStr(OLED_W - scol * 6, 23, status);

  // Row 3 — delta + compressed delta
  u8g2.setFont(u8g2_font_5x7_tf);
  char d_str[32];
  snprintf(d_str, sizeof(d_str), "d:%.2f dc:%.2f", delta, delta_c);
  u8g2.drawStr(0, 32, d_str);

  // Divider + polygram strip in lower 20px
  u8g2.drawHLine(0, 43, OLED_W);

  // Polygram in bottom 20 rows (y=44..63)
  if (g_poly_len >= 2) {
    uint32_t now = millis();
    float lo = g_poly[g_poly_head].uS, hi = lo;
    for (int i = 1; i < g_poly_len; i++) {
      float v = g_poly[(g_poly_head + i) % POLY_MAX].uS;
      if (v < lo) lo = v; if (v > hi) hi = v;
    }
    float pad   = max((hi - lo) * 0.2f, 2.0f);
    float vmin  = lo - pad;
    float range = (hi + pad) - (lo - pad);
    if (range < 1.0f) range = 1.0f;

    int prev_px = -1, prev_py = -1;
    for (int i = 0; i < g_poly_len; i++) {
      PolyPoint& p  = g_poly[(g_poly_head + i) % POLY_MAX];
      float age     = (float)(now - p.t_ms) / (float)WINDOW_MS;
      int px        = constrain((int)(OLED_W * (1.0f - age)), 0, OLED_W - 1);
      float norm_y  = 1.0f - (p.uS - vmin) / range;
      int py        = constrain(44 + (int)(norm_y * 19), 44, OLED_H - 1);
      if (prev_px >= 0) u8g2.drawLine(prev_px, prev_py, px, py);
      prev_px = px; prev_py = py;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Mode 2 — BLANK  (Detecto drums, coming soon)
// ─────────────────────────────────────────────────────────────────────────────
static void render_blank() {
  // intentionally empty
}

// ─────────────────────────────────────────────────────────────────────────────
//  display_render — call every frame from loop()
// ─────────────────────────────────────────────────────────────────────────────
void display_render(float uS, float delta, float delta_c, const DateTime& dt) {
  u8g2.clearBuffer();

  switch (g_display_mode) {
    case DISP_POLYGRAM: render_polygram();                        break;
    case DISP_NUMBERS:  render_numbers(uS, delta, delta_c, dt);  break;
    case DISP_BLANK:    render_blank();                           break;
  }

  u8g2.sendBuffer();
}
