/**
 *    ◢◣
 *   ◢■■◣
 *  ◢■■■■◣
 * emeter.ino  —  XIAO ESP32C3 + XIAO Expansion Board electropsychometer
 *
 *
 * Outputs JSON frames at 20 Hz via:
 *   • Serial  (always on, 115200 baud)
 *   • WiFi WebSocket on port 5002
 *   • BLE notify  (optional — uncomment #define USE_BLE below)
 *
 * Wiring:
 *   GSR sensor OUT  →  A0 (GPIO2 on XIAO ESP32C3)
 *   GSR sensor VCC  →  3.3V
 *   GSR sensor GND  →  GND
 *
 * Additions over v1:
 *   • OLED polygram  — 30-second rolling waveform, lifted directly from
 *                      the SVG polygram logic in index.html
 *   • PCF8563 RTC    — reads time every frame, available as g_rtc_now
 *                      (no display yet — hook it up wherever you like)
 *
 * Libraries required (Arduino Library Manager):
 *   • U8g2           by oliver
 *   • PCF8563        by Bill Greiman  (search "PCF8563 Greiman")
 *   • ArduinoWebsockets  by Gil Maimon
 *   • ArduinoJson    by Benoît Blanchon
 *
 * Board: "XIAO_ESP32C3" in Arduino IDE
 *        (Boards Manager → esp32 by Espressif)
 */

// ── Transport toggles ────────────────────────────────────────────────────────
#define USE_WIFI
// #define USE_BLE

#ifdef USE_WIFI
  const char*    WIFI_SSID = "YOUR_SSID";
  const char*    WIFI_PASS = "YOUR_PASSWORD";
  const uint16_t WS_PORT   = 5002;
#endif

// ── ADC / sensor constants ───────────────────────────────────────────────────
static const int    ADC_PIN      = A0;
static const float  GSR_RREF    = 100000.0f;
static const float  GSR_VSUPPLY = 3.3f;
static const float  ADC_MAX     = 4095.0f;
static const int    ADC_CLAMP_LO = 1;
static const int    ADC_CLAMP_HI = 4094;

// ── EMA / timing ─────────────────────────────────────────────────────────────
static const float    EMA_FAST    = 0.15f;
static const float    EMA_SLOW    = 0.005f;
static const int      WARMUP_SAMP = 100;
static const uint32_t PERIOD_US   = 50000;   // 20 Hz

// ── Polygram config (mirrors index.html) ─────────────────────────────────────
// 30-second window at 20 Hz = 600 samples max.
// OLED is 128x64; waveform lives in the bottom 20 px.
static const uint32_t WINDOW_MS = 30000;
static const int      POLY_MAX  = 600;    // 30 s x 20 Hz
static const int      OLED_W    = 128;
static const int      OLED_H    = 64;
static const int      POLY_H    = 20;              // waveform strip height (px)
static const int      POLY_Y0   = OLED_H - POLY_H; // top of strip = y 44

// ── OLED ─────────────────────────────────────────────────────────────────────
#include <U8g2lib.h>
#include <Wire.h>

// SSD1306 128x64, hardware I2C, address 0x3C (standard on expansion board)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset= */ U8X8_PIN_NONE);

// ── RTC ──────────────────────────────────────────────────────────────────────
#include <PCF8563.h>
PCF8563 rtc;

struct RtcTime {
  uint8_t  hour, minute, second;
  uint8_t  day, month;
  uint16_t year;
};
static RtcTime g_rtc_now;   // updated every frame — use anywhere

// ── Sensor state ─────────────────────────────────────────────────────────────
struct State {
  float    smoothed;
  float    baseline;
  float    delta;
  uint32_t count;
  uint32_t t0_ms;
};
static State g_state;

// ── Polygram ring buffer ──────────────────────────────────────────────────────
struct PolyPoint { uint32_t t_ms; float uS; };
static PolyPoint g_poly[POLY_MAX];
static int       g_poly_head = 0;  // index of oldest sample
static int       g_poly_len  = 0;  // number of valid samples

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers — identical to index.py
// ─────────────────────────────────────────────────────────────────────────────
float adc_to_uS(int raw) {
  raw = max(ADC_CLAMP_LO, min(ADC_CLAMP_HI, raw));
  float v = (raw / ADC_MAX) * GSR_VSUPPLY;
  float r = GSR_RREF * ((GSR_VSUPPLY / v) - 1.0f);
  return (1.0f / r) * 1e6f;
}

float compress(float x) {
  return x / (1.0f + fabsf(x) * 0.05f);
}

// ─────────────────────────────────────────────────────────────────────────────
//  next_frame — identical to index.py next_frame()
// ─────────────────────────────────────────────────────────────────────────────
String next_frame(State& s) {
  int raw = analogRead(ADC_PIN);

  s.smoothed = EMA_FAST * raw        + (1.0f - EMA_FAST) * s.smoothed;
  s.baseline = EMA_SLOW * s.smoothed + (1.0f - EMA_SLOW) * s.baseline;

  float prev_delta = s.delta;
  float raw_delta  = s.smoothed - s.baseline;
  float norm       = max(10.0f, fabsf(s.baseline));
  s.delta          = (raw_delta / norm) * 100.0f;

  float velocity = s.delta - prev_delta;
  float delta_c  = compress(s.delta);
  float vel_c    = compress(velocity);

  s.count++;

  uint32_t t_ms = millis() - s.t0_ms;
  float    uS   = adc_to_uS((int)s.smoothed);

  char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"t\":%lu,\"smooth_uS\":%.2f,\"delta\":%.3f,\"delta_c\":%.3f,\"velocity\":%.3f}",
    (unsigned long)t_ms, uS, s.delta, delta_c, vel_c);
  return String(buf);
}

// ─────────────────────────────────────────────────────────────────────────────
//  push_poly — ring-buffer insert + window eviction
//  Mirrors: polyData.push({t,uS}); polyData = polyData.filter(p => now-p.t <= WINDOW_MS)
// ─────────────────────────────────────────────────────────────────────────────
void push_poly(float uS) {
  uint32_t now = millis();

  // Write into next slot
  int slot = (g_poly_head + g_poly_len) % POLY_MAX;
  g_poly[slot] = { now, uS };
  if (g_poly_len < POLY_MAX) {
    g_poly_len++;
  } else {
    // Full — overwrite oldest
    g_poly_head = (g_poly_head + 1) % POLY_MAX;
  }

  // Evict samples older than WINDOW_MS
  while (g_poly_len > 0 && (now - g_poly[g_poly_head].t_ms) > WINDOW_MS) {
    g_poly_head = (g_poly_head + 1) % POLY_MAX;
    g_poly_len--;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  draw_polygram — renders the rolling waveform into the bottom strip
//
//  Direct port of updatePolygram() from index.html:
//    x = W * (1 - age/WINDOW_MS)          right = newest, left = oldest
//    y = H * (1 - (uS - min) / range)     top = high, bottom = low
//  Then mapped into the OLED strip POLY_Y0 .. OLED_H-1
// ─────────────────────────────────────────────────────────────────────────────
void draw_polygram() {
  if (g_poly_len < 2) return;

  uint32_t now = millis();

  // Auto-range (mirrors: lo, hi, pad, min, range in index.html)
  float lo = g_poly[g_poly_head].uS;
  float hi = lo;
  for (int i = 1; i < g_poly_len; i++) {
    float v = g_poly[(g_poly_head + i) % POLY_MAX].uS;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  float pad   = max((hi - lo) * 0.2f, 2.0f);
  float vmin  = lo - pad;
  float range = (hi + pad) - (lo - pad);
  if (range < 1.0f) range = 1.0f;

  // Draw connected line segments (oldest → newest)
  int prev_px = -1, prev_py = -1;
  for (int i = 0; i < g_poly_len; i++) {
    PolyPoint& p = g_poly[(g_poly_head + i) % POLY_MAX];

    // x: right-anchored, newest sample at far right
    float age_ratio = (float)(now - p.t_ms) / (float)WINDOW_MS;
    int px = (int)((float)OLED_W * (1.0f - age_ratio));
    px = constrain(px, 0, OLED_W - 1);

    // y: inverted (high uS = top of strip)
    float norm_y = 1.0f - (p.uS - vmin) / range;
    int py = POLY_Y0 + (int)(norm_y * (float)(POLY_H - 1));
    py = constrain(py, POLY_Y0, OLED_H - 1);

    if (prev_px >= 0) {
      u8g2.drawLine(prev_px, prev_py, px, py);
    }
    prev_px = px;
    prev_py = py;
  }

  // Horizontal rule separating info area from waveform
  u8g2.drawHLine(0, POLY_Y0 - 1, OLED_W);
}

// ─────────────────────────────────────────────────────────────────────────────
//  read_rtc — reads PCF8563 into g_rtc_now
// ─────────────────────────────────────────────────────────────────────────────
void read_rtc() {
  rtc.getTime();
  g_rtc_now.hour   = rtc.hour;
  g_rtc_now.minute = rtc.minute;
  g_rtc_now.second = rtc.second;
  rtc.getDate();
  g_rtc_now.day   = rtc.day;
  g_rtc_now.month = rtc.month;
  g_rtc_now.year  = 2000 + rtc.year;
}

// ─────────────────────────────────────────────────────────────────────────────
//  WiFi / WebSocket (unchanged from v1)
// ─────────────────────────────────────────────────────────────────────────────
#ifdef USE_WIFI
  #include <WiFi.h>
  #include <ArduinoWebsockets.h>
  using namespace websockets;

  WebsocketsServer wsServer;
  static const int MAX_CLIENTS = 4;
  WebsocketsClient wsClients[MAX_CLIENTS];
  bool             wsConnected[MAX_CLIENTS];

  void ws_init() {
    Serial.print("[wifi] connecting to "); Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
    Serial.println();
    Serial.print("[wifi] IP: "); Serial.println(WiFi.localIP());
    wsServer.listen(WS_PORT);
    Serial.print("[ws]   port "); Serial.println(WS_PORT);
    memset(wsConnected, false, sizeof(wsConnected));
  }

  void ws_accept() {
    if (wsServer.poll()) {
      WebsocketsClient client = wsServer.accept();
      for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!wsConnected[i]) {
          wsClients[i] = client; wsConnected[i] = true;
          Serial.print("[ws]   client connected (slot "); Serial.print(i); Serial.println(")");
          break;
        }
      }
    }
  }

  void ws_send(const String& json) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
      if (wsConnected[i]) {
        if (wsClients[i].available()) wsClients[i].send(json);
        else wsConnected[i] = false;
      }
    }
  }
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  BLE (opt-in, unchanged from v1)
// ─────────────────────────────────────────────────────────────────────────────
#ifdef USE_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
  #define BLE_SERVICE_UUID "12345678-1234-1234-1234-123456789abc"
  #define BLE_CHAR_UUID    "abcdefab-cdef-abcd-efab-cdefabcdefab"
  static BLECharacteristic* g_bleChar    = nullptr;
  static bool               g_bleConnected = false;
  class BLECallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*)    { g_bleConnected = true;  }
    void onDisconnect(BLEServer*) { g_bleConnected = false; }
  };
  void ble_init() {
    BLEDevice::init("emeter");
    BLEServer*  server = BLEDevice::createServer();
    server->setCallbacks(new BLECallbacks());
    BLEService* svc    = server->createService(BLE_SERVICE_UUID);
    g_bleChar = svc->createCharacteristic(BLE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    g_bleChar->addDescriptor(new BLE2902());
    svc->start();
    BLEDevice::getAdvertising()->addServiceUUID(BLE_SERVICE_UUID);
    BLEDevice::getAdvertising()->start();
  }
  void ble_send(const String& json) {
    if (!g_bleConnected) return;
    g_bleChar->setValue((uint8_t*)json.c_str(), json.length());
    g_bleChar->notify();
  }
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[emeter] booting");

  // I2C — shared bus for OLED (0x3C) and RTC (0x51)
  Wire.begin();

  // OLED
  u8g2.begin();
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.clearBuffer();
  u8g2.drawStr(0, 10, "emeter booting...");
  u8g2.sendBuffer();

  // RTC
  rtc.begin();
  // ── Set time once, then comment out and reflash ──────────────────────────
  // rtc.setDateTime(2025, 6, 1, 12, 0, 0);  // YYYY, M, D, H, Min, Sec
  // ─────────────────────────────────────────────────────────────────────────
  read_rtc();
  Serial.printf("[rtc]  %04d-%02d-%02d %02d:%02d:%02d\n",
    g_rtc_now.year, g_rtc_now.month, g_rtc_now.day,
    g_rtc_now.hour, g_rtc_now.minute, g_rtc_now.second);

  // ADC — 12-bit, full 3.3V range
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Seed state from a real ADC read
  int seed = analogRead(ADC_PIN);
  g_state = { (float)seed, (float)seed, 0.0f, 0, millis() };

  // Warmup — let EMAs settle silently
  Serial.println("[emeter] warming up...");
  for (int i = 0; i < WARMUP_SAMP; i++) {
    next_frame(g_state);
    delayMicroseconds(PERIOD_US);
  }
  g_state.count = 0;
  g_state.t0_ms = millis();
  Serial.println("[emeter] warmup done");

#ifdef USE_WIFI
  ws_init();
#endif
#ifdef USE_BLE
  ble_init();
#endif

  Serial.println("[emeter] streaming at 20 Hz");
}

// ─────────────────────────────────────────────────────────────────────────────
//  loop — 20 Hz fixed-rate
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t tick = micros();

#ifdef USE_WIFI
  ws_accept();
#endif

  // Compute frame
  String json = next_frame(g_state);
  float  uS   = adc_to_uS((int)g_state.smoothed);

  // Update polygram ring buffer
  push_poly(uS);

  // Read RTC — g_rtc_now is fresh after this; add display calls when ready
  read_rtc();

  // ── OLED render ────────────────────────────────────────────────────────────
  u8g2.clearBuffer();

  // Row 1 — µS value, bold
  u8g2.setFont(u8g2_font_7x14B_tr);
  char us_str[20];
  snprintf(us_str, sizeof(us_str), "%.2f uS", uS);
  u8g2.drawStr(0, 13, us_str);

  // Row 2 — delta and velocity, small
  u8g2.setFont(u8g2_font_5x7_tr);
  char dv_str[32];
  snprintf(dv_str, sizeof(dv_str), "d:%.2f v:%.2f", g_state.delta, compress(g_state.delta));
  u8g2.drawStr(0, 24, dv_str);

  // Bottom strip — polygram waveform
  draw_polygram();

  u8g2.sendBuffer();
  // ──────────────────────────────────────────────────────────────────────────

  // Transmit
  Serial.println(json);
#ifdef USE_WIFI
  ws_send(json);
#endif
#ifdef USE_BLE
  ble_send(json);
#endif

  // Fixed-rate sleep
  uint32_t elapsed = micros() - tick;
  if (elapsed < PERIOD_US) delayMicroseconds(PERIOD_US - elapsed);
}
