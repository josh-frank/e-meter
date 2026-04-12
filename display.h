/**
 * display.h
 *
 * All OLED output for e-meter.ino.
 * Cycle display modes with the button on the XIAO Expansion Board (D1).
 *
 * Modes:
 *   0  DISP_POLYGRAM  — full-screen 30-second rolling waveform
 *   1  DISP_NUMBERS   — µS + delta + clock + status icons
 *   2  DISP_DETECTO   — two uS drums + rise-set-fall delta lane
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
//  Three lanes, 128×64, each ~21 px tall.
//
//  Lane 0 (top)    — SLOW drum, uS, 1 rotation = 1 µS
//                    Coarse position. Moves steadily with conductance level.
//
//  Lane 1 (middle) — FAST drum, uS, 1 rotation = 0.1 µS
//                    Fine position. More responsive to small changes.
//
//  Lane 2 (bottom) — RISE / SET / FALL tape, driven by delta_c
//                    Does not rotate — it is a fixed tape shifted by delta_c.
//                    Centre (x=64) = SET (delta_c ≈ 0, quiet baseline).
//                    Positive delta_c (rise) shifts tape left  → read point
//                    moves right of SET toward RISE labels.
//                    Negative delta_c (fall) shifts tape right → read point
//                    moves left of SET toward FALL labels.
//                    Hard-clamped: tape never scrolls past the display edges,
//                    so sustained events just pin at the stop.
//                    Tick density increases toward SET (centre) and thins
//                    toward the extremes — finer resolution near null,
//                    coarser at the stops.
//
//  Hairline at x = OLED_W/2 is the read point for all three lanes.
//
//  Tuning knobs:
//    DETECTO_RANGE_SLOW / _FAST   — µS per full drum rotation
//    DETECTO_*_MAJ/MID/MIN        — tick spacings in px for the two uS drums
//    DETECTO_RSF_SCALE            — how many px the RSF tape moves per unit
//                                   of delta_c  (larger = more sensitive)
//    DETECTO_RSF_CLAMP            — max tape travel in px from centre
//                                   (should be ≤ OLED_W/2 to stay on screen)
// ─────────────────────────────────────────────────────────────────────────────

// µS per full drum rotation - rull range r ≈ 17.0
static const float DETECTO_RANGE_SLOW =  34.0f;   // 2r
static const float DETECTO_RANGE_FAST =  9.444f;  // 5r/9

// Tick spacings (px): major, mid, minor — for the two uS drums
static const int DETECTO_SLOW_MAJ = 30, DETECTO_SLOW_MID = 15, DETECTO_SLOW_MIN = 5;
static const int DETECTO_FAST_MAJ = 64, DETECTO_FAST_MID =  8, DETECTO_FAST_MIN = 4;

// Virtual tape length — must be >> OLED_W to avoid modulo wrap artefacts
static const int DETECTO_TAPE_W = 2000;

// Rise-Set-Fall lane tuning
// SCALE: px of tape travel per unit of delta_c.
//   delta_c is compress(delta) so it's bounded roughly ±20 in practice.
//   At scale=3, a delta_c of 20 moves the tape 60 px — just past the stop.
static const float DETECTO_RSF_SCALE = 3.0f;
// CLAMP: hard stop — tape travel capped at ±this many px from centre.
//   Keep ≤ 60 so the tape never scrolls completely off screen.
static const int   DETECTO_RSF_CLAMP = 58;

// ── Detecto spring physics ────────────────────────────────────────────────────
//  Damped spring matching the galvanometer model in animated.html.
//  State is kept in tape-pixel space (same unit as the drum offset).
//
//  SPRING_K   — stiffness: fraction of remaining error applied per reference
//               frame. 0.0325 (from HTML) gives a gentle, slightly lazy swing.
//               Raise toward ~0.10 for a snappier needle.
//  SPRING_D   — damping multiplier applied to velocity each frame.
//               0.925 (from HTML) = light damping, one or two overshoots.
//               Lower toward ~0.80 for a dead-beat, no-overshoot response.
//  SPRING_TARGET_FPS — reference frame-rate the K/D constants assume.
//               dt scaling normalises everything so loop speed doesn't matter.
//  SPRING_BOOST_K / BOOST_CAP — on large target jumps the HTML adds a
//               one-shot impulse proportional to the jump size; these mirror
//               those coefficients.
static const float DETECTO_SPRING_K          = 0.0325f;
static const float DETECTO_SPRING_D          = 0.925f;
static const float DETECTO_SPRING_TARGET_FPS = 60.0f;
static const float DETECTO_SPRING_BOOST_K    = 0.125f;
static const float DETECTO_SPRING_BOOST_CAP  = 2.0f;

struct DetectoSpring {
  float pos;       // current smoothed tape offset (px)
  float vel;       // current velocity (px / reference-frame)
  float target;    // last requested target offset (px)
};

static DetectoSpring g_spring_slow = {0.0f, 0.0f, 0.0f};
static DetectoSpring g_spring_fast = {0.0f, 0.0f, 0.0f};

// Advance one spring by dt_ms milliseconds toward target_px.
// Call once per display_render() for each drum lane.
static float detecto_spring_step(DetectoSpring& s, float target_px,
                                  uint32_t dt_ms) {
  // How many reference frames elapsed this tick?
  float frames = (float)dt_ms * DETECTO_SPRING_TARGET_FPS / 1000.0f;
  if (frames < 0.001f) frames = 0.001f;   // guard against dt=0 on first call

  float new_target = target_px;

  // One-shot velocity boost on target jump (mirrors updateMeterTarget in HTML)
  float jump = new_target - s.target;
  if (fabsf(jump) > 0.5f) {
    float boost = min(fabsf(jump) * DETECTO_SPRING_BOOST_K * DETECTO_SPRING_BOOST_K,
                      DETECTO_SPRING_BOOST_CAP);
    s.vel += jump * DETECTO_SPRING_BOOST_K * boost;
  }
  s.target = new_target;

  // Integrate spring over `frames` reference steps.
  // Using exact per-step iteration (max 4) keeps feel identical to the HTML
  // at any real loop rate without the cost of a full Euler sub-step loop.
  int steps = (int)constrain(frames + 0.5f, 1, 4);
  float k_scaled = DETECTO_SPRING_K * frames / (float)steps;
  float d_scaled = powf(DETECTO_SPRING_D, frames / (float)steps);

  for (int i = 0; i < steps; i++) {
    float delta = s.target - s.pos;
    s.vel  = (s.vel + delta * k_scaled) * d_scaled;
    s.pos += s.vel;
  }

  return s.pos;
}

// ── Shared: draw one scrolling tick-mark drum lane ────────────────────────────
//   y0, h         — top pixel and height of this lane
//   uS, range     — current reading and µS per full rotation
//   majSp/midSp/minSp — tick spacings in px
//   invert        — if true, ticks grow DOWN from the top border (inward).
//                   Used for lane 1 (FAST) so the two drums bracket the
//                   hairline symmetrically and large reads are easy to spot.
static void detecto_draw_lane(int y0, int h,
                               float uS, float range,
                               int majSp, int midSp, int minSp,
                               bool invert = false) {
  int offset    = (int)((uS / range) * (float)DETECTO_TAPE_W) % DETECTO_TAPE_W;
  int tape_left = offset - OLED_W / 2;

  // Minor ticks
  for (int tx = (tape_left / minSp) * minSp; tx <= tape_left + OLED_W; tx += minSp) {
    if (tx % midSp == 0) continue;
    int sx = tx - tape_left;
    if (sx < 0 || sx >= OLED_W) continue;
    int th = h / 4;
    int ty = invert ? y0 : y0 + h - th;
    u8g2.drawVLine(sx, ty, th);
  }
  // Mid ticks
  for (int tx = (tape_left / midSp) * midSp; tx <= tape_left + OLED_W; tx += midSp) {
    if (tx % majSp == 0) continue;
    int sx = tx - tape_left;
    if (sx < 0 || sx >= OLED_W) continue;
    int th = (h * 55) / 100;
    int ty = invert ? y0 : y0 + h - th;
    u8g2.drawVLine(sx, ty, th);
  }
  // Major ticks
  for (int tx = (tape_left / majSp) * majSp; tx <= tape_left + OLED_W; tx += majSp) {
    int sx = tx - tape_left;
    if (sx < 0 || sx >= OLED_W) continue;
    u8g2.drawVLine(sx, y0, h);
  }

  u8g2.drawHLine(0, y0, OLED_W);
}

// ── Rise-Set-Fall lane ────────────────────────────────────────────────────────
//   y0, h     — top pixel and height of this lane
//   delta_c   — compressed delta; positive = rise, negative = fall, 0 = SET
//
//  The tape has three zones of tick density:
//    outer 25% of travel (near stops) — major ticks only, very sparse
//    middle 50%                        — major + minor
//    inner 25% (near SET)             — major + mid + minor (densest)
//  This makes the lane most readable near null and deliberately coarse
//  at the extremes where exact position matters less.
//
//  A double-height major tick marks SET at tape centre (x=0 on the tape).
static void detecto_draw_rsf(int y0, int h, float delta_c) {
  // Tape shift: positive delta_c → tape moves left → hairline reads right of SET
  int shift = constrain((int)(delta_c * DETECTO_RSF_SCALE),
                         -DETECTO_RSF_CLAMP, DETECTO_RSF_CLAMP);

  // tape_x=0 is SET. screen_x = OLED_W/2 - shift + tape_x
  // So the tape origin (SET mark) sits at screen_x = OLED_W/2 - shift.
  int set_sx = OLED_W / 2 - shift;

  // Tick layout on the tape (symmetric around SET at tape_x=0):
  //   Major every 20 px
  //   Mid   every 10 px  (only within ±30 px of SET)
  //   Minor every  5 px  (only within ±15 px of SET)
  const int MAJ = 20, MID = 10, MIN = 5;
  const int MID_ZONE = 30, MIN_ZONE = 15;

  // Scan tape_x range that maps to screen [0, OLED_W)
  int tape_lo = -set_sx;              // tape_x at screen_x = 0
  int tape_hi = tape_lo + OLED_W - 1;

  // Minor ticks (innermost zone)
  for (int tx = (tape_lo / MIN) * MIN; tx <= tape_hi; tx += MIN) {
    if (tx % MID == 0) continue;
    if (abs(tx) > MIN_ZONE) continue;
    int sx = set_sx + tx;
    if (sx < 0 || sx >= OLED_W) continue;
    u8g2.drawVLine(sx, y0 + h - h / 4, h / 4);
  }
  // Mid ticks
  for (int tx = (tape_lo / MID) * MID; tx <= tape_hi; tx += MID) {
    if (tx % MAJ == 0) continue;
    if (abs(tx) > MID_ZONE) continue;
    int sx = set_sx + tx;
    if (sx < 0 || sx >= OLED_W) continue;
    int th = (h * 55) / 100;
    u8g2.drawVLine(sx, y0 + h - th, th);
  }
  // Major ticks — full range
  for (int tx = (tape_lo / MAJ) * MAJ; tx <= tape_hi; tx += MAJ) {
    int sx = set_sx + tx;
    if (sx < 0 || sx >= OLED_W) continue;
    if (tx == 0) {
      // SET mark — double tick: full height + a pip at top
      u8g2.drawVLine(sx,     y0,     h);
      u8g2.drawVLine(sx - 1, y0,     h / 2);   // left shoulder
      u8g2.drawVLine(sx + 1, y0,     h / 2);   // right shoulder
    } else {
      u8g2.drawVLine(sx, y0, h);
    }
  }

  u8g2.drawHLine(0, y0, OLED_W);
}

static void render_detecto(float uS, float delta_c) {
  const int laneH  = OLED_H / 3;          // 21 px
  const int lane2H = OLED_H - laneH * 2;  // 22 px — bottom lane gets the spare pixel

  // ── dt measurement ──────────────────────────────────────────────────────────
  //  Track wall-clock time between renders so the spring integrator is
  //  loop-rate independent. Static so it persists across calls.
  static uint32_t s_last_render_ms = 0;
  uint32_t now_ms = millis();
  uint32_t dt_ms  = (s_last_render_ms == 0) ? 16 : (now_ms - s_last_render_ms);
  dt_ms = constrain(dt_ms, 1, 200);  // clamp: ignore stalls > 200 ms
  s_last_render_ms = now_ms;

  // ── Spring targets (tape-pixel space) ───────────────────────────────────────
  //  Convert raw uS → the same offset integer detecto_draw_lane would use,
  //  then let the spring chase that value.
  float slow_target = (uS / DETECTO_RANGE_SLOW) * (float)DETECTO_TAPE_W;
  float fast_target = (uS / DETECTO_RANGE_FAST) * (float)DETECTO_TAPE_W;

  float slow_pos = detecto_spring_step(g_spring_slow, slow_target, dt_ms);
  float fast_pos = detecto_spring_step(g_spring_fast, fast_target, dt_ms);

  // ── Draw lanes with smoothed positions ──────────────────────────────────────
  //  Pass smoothed position as a synthetic uS whose offset equals slow/fast_pos.
  //  Multiplying back by range/TAPE_W recovers the right fractional uS value
  //  without touching detecto_draw_lane's internal maths.
  float slow_uS_smooth = fmodf(slow_pos, (float)DETECTO_TAPE_W)
                         / (float)DETECTO_TAPE_W * DETECTO_RANGE_SLOW;
  float fast_uS_smooth = fmodf(fast_pos, (float)DETECTO_TAPE_W)
                         / (float)DETECTO_TAPE_W * DETECTO_RANGE_FAST;

  detecto_draw_lane(0,         laneH,  slow_uS_smooth, DETECTO_RANGE_SLOW,
                    DETECTO_SLOW_MAJ, DETECTO_SLOW_MID, DETECTO_SLOW_MIN,
                    false);   // lane 0: ticks grow UP from bottom border (outward)
  detecto_draw_lane(laneH,     laneH,  fast_uS_smooth, DETECTO_RANGE_FAST,
                    DETECTO_FAST_MAJ, DETECTO_FAST_MID, DETECTO_FAST_MIN,
                    true);    // lane 1: ticks grow DOWN from top border (inward)
  detecto_draw_rsf (laneH * 2, lane2H, delta_c);

  // Hairline — drawn last, over all tick marks
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
    case DISP_DETECTO:  render_detecto(uS, delta_c);             break;
    // case DISP_PLACEHOLDER: render_placeholder();               break;
  }

  u8g2.sendBuffer();
}