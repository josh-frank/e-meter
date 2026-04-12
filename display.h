/**
 * display.h
 *
 * All OLED output for e-meter.ino.
 * Cycle display modes with the button on the XIAO Expansion Board (D1).
 *
 * Modes:
 *   0  DISP_POLYGRAM  — full-screen 30-second rolling waveform
 *   1  DISP_NUMBERS   — µS + delta + clock + status icons
 *   2  DISP_DETECTO   — three-drum Detecto-style continuous dial
 *  (3) DISP_PLACEHOLDER — commented out; uncomment to add a new mode
 *
 * Public API (call from e-meter.ino):
 *   display_splash()               — boot logo, call after u8g2.begin()
 *   display_setup()                — wires up button ISR, call after display_splash()
 *   display_render(uS, d, dc, dt)  — render current mode, call every loop()
 *
 * Externs required in e-meter.ino (must NOT be static):
 *   PolyPoint g_poly[POLY_MAX];
 *   int       g_poly_head;
 *   int       g_poly_len;
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

// ── Externs — defined in e-meter.ino ─────────────────────────────────────────
extern U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2;
extern PolyPoint g_poly[];
extern int       g_poly_head;
extern int       g_poly_len;

// Transport state externs — conditionally compiled to match e-meter.ino
#ifdef USE_SD
  extern bool g_recording;
#endif
#ifdef USE_BLE
  extern bool g_bleConnected;
#endif

// DateTime forward declaration — full definition comes from PCF8563.h
struct DateTime;

// ── Button ────────────────────────────────────────────────────────────────────
static const uint8_t  BTN_PIN      = D1;
static const uint32_t BTN_DEBOUNCE = 50;   // ms

// ── Display mode ──────────────────────────────────────────────────────────────
#define DISP_POLYGRAM    0
#define DISP_NUMBERS     1
#define DISP_DETECTO     2
// #define DISP_PLACEHOLDER 3   // ← uncomment + set DISP_MODE_MAX 4 to add a mode
#define DISP_MODE_MAX    3

volatile uint8_t  g_display_mode = DISP_POLYGRAM;
volatile uint32_t g_btn_last_ms  = 0;

// ── ISR ───────────────────────────────────────────────────────────────────────
void IRAM_ATTR btn_isr() {
  uint32_t now = millis();
  if (now - g_btn_last_ms > BTN_DEBOUNCE) {
    g_display_mode = (g_display_mode + 1) % DISP_MODE_MAX;
    g_btn_last_ms  = now;
  }
}

// ── display_splash — boot logo ────────────────────────────────────────────────
//  Centred text, 5x7 font (each char is 6 px wide including 1 px gap).
//  x = (128 - numChars * 6) / 2
void display_splash() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tr);

  u8g2.drawStr(43,  8, "HAGGARD");
  u8g2.drawStr(28, 16, "ELECTROMETER");
  u8g2.drawStr( 2, 24, "FOR USE IN SHENANIGANS");
  u8g2.drawStr( 2, 32, "AMERICAN - MARK 0-POLO");

  u8g2.drawHLine(0, 36, OLED_W);
  u8g2.drawStr(0, 45, "booting...");

  u8g2.sendBuffer();
}

// ── display_setup — call once after display_splash() ─────────────────────────
void display_setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), btn_isr, FALLING);
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
//  Layout (128x64):
//    y=0..13   µS reading (large font)
//    y=14..23  clock (left) + transport status icons (right)
//    y=24..32  delta + compressed delta
//    y=43      divider
//    y=44..63  polygram strip (20 px)
//
//  Status icons (5x7 font, right-aligned):
//    W / w  — WiFi connected / offline
//    B / b  — BLE connected / advertising
//    * / -  — SD recording / mounted idle
//
//  µ glyph note:
//    drawStr() treats strings as Latin-1, so the UTF-8 µ (0xC2 0xB5)
//    renders as two glyphs. Use drawUTF8() with a _tf font instead.
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

  // Divider + polygram strip (y=44..63)
  u8g2.drawHLine(0, 43, OLED_W);

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
      PolyPoint& p = g_poly[(g_poly_head + i) % POLY_MAX];
      float age    = (float)(now - p.t_ms) / (float)WINDOW_MS;
      int px       = constrain((int)(OLED_W * (1.0f - age)), 0, OLED_W - 1);
      float norm_y = 1.0f - (p.uS - vmin) / range;
      int py       = constrain(44 + (int)(norm_y * 19), 44, OLED_H - 1);
      if (prev_px >= 0) u8g2.drawLine(prev_px, prev_py, px, py);
      prev_px = px; prev_py = py;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Mode 2 — DETECTO
//
//  Three-drum continuous dial, inspired by Detecto beam scales and jump-hour
//  watches. Each drum is a virtual tape of tick marks scrolled by the
//  fractional position of uS within that drum's range. The centre hairline
//  is the read point; the tape scrolls left as uS increases.
//
//  Layout (128×64, three equal lanes of ~21 px):
//    Lane 0 (top)    — slow drum,  1 rotation = DETECTO_RANGE_SLOW µS
//                      coarse position; rarely wraps in normal use
//    Lane 1 (middle) — med drum,   1 rotation = DETECTO_RANGE_MED µS
//                      the "working" dial you actually watch
//    Lane 2 (bottom) — fast drum,  1 rotation = DETECTO_RANGE_FAST µS
//                      micro-fluctuations; twitchy and alive
//
//  Tick geometry (drawn from bottom of each lane upward):
//    Major tick  — full lane height
//    Mid tick    — 55% height
//    Minor tick  — 25% height
//
//  Tuning: adjust the DETECTO_RANGE_* and spacing constants below.
// ─────────────────────────────────────────────────────────────────────────────

// µS per full drum rotation
static const float DETECTO_RANGE_SLOW = 17.0f;   // coarse — full GSR range
static const float DETECTO_RANGE_MED  =  1.0f;   // working dial
static const float DETECTO_RANGE_FAST =  0.1f;   // fine detail

// Tick spacings (px): major, mid, minor
static const int DETECTO_SLOW_MAJ = 20, DETECTO_SLOW_MID = 10, DETECTO_SLOW_MIN = 5;
static const int DETECTO_MED_MAJ  = 20, DETECTO_MED_MID  = 10, DETECTO_MED_MIN  = 5;
static const int DETECTO_FAST_MAJ = 16, DETECTO_FAST_MID =  8, DETECTO_FAST_MIN = 4;

// Virtual tape length (px) — must be comfortably > OLED_W; wraps modulo this
static const int DETECTO_TAPE_W = 2000;

// Draw one drum lane.
//   y0, h         — top pixel and pixel height of this lane
//   uS, range     — current reading and µS per full rotation
//   majSp/midSp/minSp — tick spacing in px
static void detecto_draw_lane(int y0, int h,
                               float uS, float range,
                               int majSp, int midSp, int minSp) {
  // How far along the tape are we? Centre that position on x = OLED_W/2.
  int offset   = (int)((uS / range) * (float)DETECTO_TAPE_W) % DETECTO_TAPE_W;
  int tape_left = offset - OLED_W / 2;   // tape coord at screen x = 0

  // Minor ticks (drawn first — brighter marks overwrite)
  for (int tx = (tape_left / minSp) * minSp; tx <= tape_left + OLED_W; tx += minSp) {
    if (tx % midSp == 0) continue;       // handled below
    int sx = tx - tape_left;
    if (sx < 0 || sx >= OLED_W) continue;
    int th = h / 4;
    u8g2.drawVLine(sx, y0 + h - th, th);
  }
  // Mid ticks
  for (int tx = (tape_left / midSp) * midSp; tx <= tape_left + OLED_W; tx += midSp) {
    if (tx % majSp == 0) continue;       // handled below
    int sx = tx - tape_left;
    if (sx < 0 || sx >= OLED_W) continue;
    int th = (h * 55) / 100;
    u8g2.drawVLine(sx, y0 + h - th, th);
  }
  // Major ticks
  for (int tx = (tape_left / majSp) * majSp; tx <= tape_left + OLED_W; tx += majSp) {
    int sx = tx - tape_left;
    if (sx < 0 || sx >= OLED_W) continue;
    u8g2.drawVLine(sx, y0, h);
  }

  // Lane top-edge divider
  u8g2.drawHLine(0, y0, OLED_W);
}

static void render_detecto(float uS) {
  const int laneH = OLED_H / 3;                    // 21 px; last lane takes remainder
  const int lane2H = OLED_H - laneH * 2;           // 22 px

  detecto_draw_lane(0,          laneH,  uS, DETECTO_RANGE_SLOW,
                    DETECTO_SLOW_MAJ, DETECTO_SLOW_MID, DETECTO_SLOW_MIN);
  detecto_draw_lane(laneH,      laneH,  uS, DETECTO_RANGE_MED,
                    DETECTO_MED_MAJ,  DETECTO_MED_MID,  DETECTO_MED_MIN);
  detecto_draw_lane(laneH * 2,  lane2H, uS, DETECTO_RANGE_FAST,
                    DETECTO_FAST_MAJ, DETECTO_FAST_MID, DETECTO_FAST_MIN);

  // Hairline — drawn last so it sits on top of all tick marks
  u8g2.drawVLine(OLED_W / 2, 0, OLED_H);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Mode 3 — PLACEHOLDER  (commented out)
//
//  To activate:
//    1. Uncomment #define DISP_PLACEHOLDER 3 above
//    2. Change DISP_MODE_MAX to 4
//    3. Uncomment render_placeholder() and the case in display_render() below
// ─────────────────────────────────────────────────────────────────────────────
// static void render_placeholder() {
//   // your new mode here
// }

// ─────────────────────────────────────────────────────────────────────────────
//  display_render — call every frame from loop()
// ─────────────────────────────────────────────────────────────────────────────
void display_render(float uS, float delta, float delta_c, const DateTime& dt) {
  u8g2.clearBuffer();

  switch (g_display_mode) {
    case DISP_POLYGRAM: render_polygram();                        break;
    case DISP_NUMBERS:  render_numbers(uS, delta, delta_c, dt);  break;
    case DISP_DETECTO:  render_detecto(uS);                      break;
    // case DISP_PLACEHOLDER: render_placeholder();               break;
  }

  u8g2.sendBuffer();
}