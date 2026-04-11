/**
 *   ◢◣
 *  ◢■■◣
 * ◢■■■■◣
 * e-meter.ino
 *
 * Outputs JSON frames at 20 Hz via:
 *   • Serial  (always on, 115200 baud)
 *   • WiFi WebSocket on port 5002
 *   • BLE notify
 *   • SD card  (SRT file in /data/)
 *
 * Wiring:
 *   GSR sensor OUT  →  A0 (GPIO2 on XIAO ESP32S3)
 *   GSR sensor VCC  →  3.3V
 *   GSR sensor GND  →  GND
 *
 * Libraries required (Arduino Library Manager):
 *   • U8g2              by olikraus
 *   • ArduinoWebsockets by Gil Maimon
 *   • ArduinoJson       by Benoît Blanchon
 *
 * Local files (in sketch folder):
 *   • PCF8563.h / PCF8563.cpp
 *   • secrets.h
 *
 * Board: "XIAO_ESP32S3" in Arduino IDE  ← plain S3, not S3 Plus
 *        (Boards Manager → esp32 by Espressif)
 *        Works for both plain S3 and S3 Sense (cam/mic variant).
 *
 * Clock sync
 *   When USE_WIFI is enabled, the sketch syncs the RTC automatically
 *   via SNTP (pool.ntp.org) every time WiFi connects. No tools, no
 *   scripts, no reflashing — just enable WiFi and it self-corrects.
 *   The coin cell keeps time when WiFi is off.
 *
 * RTC time anchor
 *   On boot the sketch prints one line to Serial:
 *     [rtc] unix_at_boot=<epoch> t_ms_at_boot=<millis>
 *   The client can reconstruct unix time for any frame as:
 *     unix = unix_at_boot + (frame.t - t_ms_at_boot) / 1000
 *
 * SD recording
 *   When USE_SD is defined, flipping the toggle at the top of the sketch
 *   (SD_RECORD_ON_BOOT) starts recording immediately on boot, exactly like
 *   passing --record to index.py.  Files are written to /data/ on the card
 *   as  eda_YYYYMMDD_HHMMSS.srt  (falls back to eda_<millis>.srt if the
 *   RTC has no valid time).  Frames are buffered and flushed every
 *   SD_FLUSH_FRAMES frames (default 20 = 1 second) to keep SPI happy.
 *   Requires: XIAO ESP32S3 Sense + Seeed Expansion Board  (CS = GPIO 21)
 *   Card must be FAT32 formatted, max 32 GB.
 */

// ── Includes ─────────────────────────────────────────────────────────────────
#include <Wire.h>
#include <U8g2lib.h>
#include "PCF8563.h"
#include "secrets.h"

// ── Transport toggles ────────────────────────────────────────────────────────
#define USE_WIFI
// #define USE_BLE
// #define USE_SD

// ── SD recording toggle ───────────────────────────────────────────────────────
//   Only meaningful when USE_SD is defined.
//   Set true  → recording starts on boot  (like --record flag in index.py)
//   Set false → SD is mounted but recording is off until you add runtime logic
#ifdef USE_SD
  #define SD_RECORD_ON_BOOT true
#endif

#ifdef USE_WIFI
  #include <WiFi.h>
  #include <ArduinoWebsockets.h>
  #include <esp_sntp.h>               // built-in, no install needed
  using namespace websockets;

  const char*    WIFI_SSID  = SECRET_SSID;
  const char*    WIFI_PASS  = SECRET_PASS;
  const uint16_t WS_PORT    = SECRET_PORT;
  const char*    NTP_SERVER = "pool.ntp.org";
  const char*    TZ_INFO    = "UTC0";  // change for local time, e.g.:
                                       // "EST5EDT,M3.2.0,M11.1.0"      US Eastern
                                       // "GMT0BST,M3.5.0/1,M10.5.0"    UK
                                       // "CET-1CEST,M3.5.0,M10.5.0/3"  Central Europe
#endif

// ── ADC / sensor constants ───────────────────────────────────────────────────
static const int   ADC_PIN      = A0;
static const float GSR_RREF     = 100000.0f;
static const float GSR_VSUPPLY  = 3.3f;
static const float ADC_MAX      = 4095.0f;
static const int   ADC_CLAMP_LO = 1;
static const int   ADC_CLAMP_HI = 4094;

// ── EMA / timing ─────────────────────────────────────────────────────────────
static const float    EMA_FAST    = 0.15f;
static const float    EMA_SLOW    = 0.005f;
static const int      WARMUP_SAMP = 100;
static const uint32_t PERIOD_US   = 50000;   // 20 Hz

// ── Polygram config ───────────────────────────────────────────────────────────
static const uint32_t WINDOW_MS = 30000;     // 30-second rolling window
static const int      POLY_MAX  = 600;       // 30 s x 20 Hz
static const int      OLED_W    = 128;
static const int      OLED_H    = 64;
static const int      POLY_H    = 20;
static const int      POLY_Y0   = OLED_H - POLY_H;

// ── OLED ─────────────────────────────────────────────────────────────────────
// SSD1306/SSD1315 — U8g2 default 7-bit address 0x3C is correct.
// Do NOT call setI2CAddress(0x78) — that is the 8-bit form and will
// cause U8g2 to talk to the wrong address, leaving the display blank.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);

// ── RTC ──────────────────────────────────────────────────────────────────────
PCF8563 rtc;

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
static int       g_poly_head = 0;
static int       g_poly_len  = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  SD / SRTWriter
// ─────────────────────────────────────────────────────────────────────────────
#ifdef USE_SD
  #include <FS.h>
  #include <SD.h>
  #include <SPI.h>

  static const int SD_CS_PIN      = 21;    // GPIO21 on XIAO ESP32S3 Sense
  static const int SD_FLUSH_FRAMES = 20;   // flush every 1 s (20 frames × 50 ms)

  static bool   g_sd_ok      = false;  // card mounted successfully
  static bool   g_recording  = false;  // currently writing frames
  static File   g_srt_file;
  static uint32_t g_srt_index    = 0;  // SRT block counter (1-based)
  static int      g_unflushed    = 0;  // frames written since last flush

  // ── ms → SRT timecode  HH:MM:SS,mmm ─────────────────────────────────────
  static void ms_to_tc(char* buf, size_t len, uint32_t ms) {
    uint32_t h   = ms / 3600000; ms %= 3600000;
    uint32_t m   = ms / 60000;   ms %= 60000;
    uint32_t s   = ms / 1000;    ms %= 1000;
    snprintf(buf, len, "%02lu:%02lu:%02lu,%03lu",
             (unsigned long)h, (unsigned long)m,
             (unsigned long)s, (unsigned long)ms);
  }

  // ── Open a new SRT file on the card ──────────────────────────────────────
  static void sd_open_file() {
    if (!g_sd_ok) return;

    // mkdir /data if needed
    if (!SD.exists("/data")) SD.mkdir("/data");

    // Build filename from RTC; fall back to millis if RTC not set
    char path[48];
    DateTime dt = rtc.getDateTime();
    if (dt.year >= 2024) {
      snprintf(path, sizeof(path), "/data/eda_%04d%02d%02d_%02d%02d%02d.srt",
               dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
    } else {
      snprintf(path, sizeof(path), "/data/eda_%lu.srt", (unsigned long)millis());
    }

    g_srt_file = SD.open(path, FILE_WRITE);
    if (!g_srt_file) {
      Serial.print("[sd] failed to open "); Serial.println(path);
      return;
    }

    g_srt_index   = 0;
    g_unflushed   = 0;
    g_recording   = true;
    Serial.print("[sd] recording → "); Serial.println(path);
  }

  // ── Write one EDA frame as an SRT block ──────────────────────────────────
  //   Format matches fourth_session.srt exactly:
  //     <index>
  //     HH:MM:SS,mmm --> HH:MM:SS,mmm
  //     <uS value>
  //     <blank line>
  static void sd_write_frame(uint32_t t_ms, float uS) {
    if (!g_recording || !g_srt_file) return;

    g_srt_index++;

    char tc_start[16], tc_end[16];
    ms_to_tc(tc_start, sizeof(tc_start), t_ms);
    ms_to_tc(tc_end,   sizeof(tc_end),   t_ms + (uint32_t)(PERIOD_US / 1000));

    char block[80];
    int n = snprintf(block, sizeof(block),
                     "%lu\n%s --> %s\n%.4f\n\n",
                     (unsigned long)g_srt_index,
                     tc_start, tc_end, uS);
    g_srt_file.write((uint8_t*)block, n);

    g_unflushed++;
    if (g_unflushed >= SD_FLUSH_FRAMES) {
      g_srt_file.flush();
      g_unflushed = 0;
    }
  }

  // ── Stop recording and close the file ────────────────────────────────────
  static void sd_stop() {
    if (!g_recording) return;
    g_srt_file.flush();
    g_srt_file.close();
    g_recording = false;
    Serial.println("[sd] recording stopped");
  }

  // ── Mount card and optionally start recording ─────────────────────────────
  static void sd_init(bool record_on_boot) {
    if (!SD.begin(SD_CS_PIN)) {
      Serial.println("[sd] mount failed — check card (FAT32, ≤32 GB)");
      return;
    }
    g_sd_ok = true;
    Serial.printf("[sd] card mounted  (%.1f MB free)\n",
                  (float)(SD.totalBytes() - SD.usedBytes()) / 1048576.0f);
    if (record_on_boot) sd_open_file();
  }
#endif  // USE_SD

// ─────────────────────────────────────────────────────────────────────────────
//  Helpers
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
//  next_frame
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
//  push_poly
// ─────────────────────────────────────────────────────────────────────────────
void push_poly(float uS) {
  uint32_t now = millis();

  int slot = (g_poly_head + g_poly_len) % POLY_MAX;
  g_poly[slot] = { now, uS };
  if (g_poly_len < POLY_MAX) {
    g_poly_len++;
  } else {
    g_poly_head = (g_poly_head + 1) % POLY_MAX;
  }

  while (g_poly_len > 0 && (now - g_poly[g_poly_head].t_ms) > WINDOW_MS) {
    g_poly_head = (g_poly_head + 1) % POLY_MAX;
    g_poly_len--;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  draw_polygram
// ─────────────────────────────────────────────────────────────────────────────
void draw_polygram() {
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
    int py = constrain(POLY_Y0 + (int)(norm_y * (POLY_H - 1)), POLY_Y0, OLED_H - 1);

    if (prev_px >= 0) u8g2.drawLine(prev_px, prev_py, px, py);
    prev_px = px;
    prev_py = py;
  }

  u8g2.drawHLine(0, POLY_Y0 - 1, OLED_W);
}

// ─────────────────────────────────────────────────────────────────────────────
//  sntp_sync_rtc
//  Called automatically by the ESP32 SNTP stack once the time is confirmed.
//  Pushes the synced UTC time into the PCF8563 hardware RTC.
// ─────────────────────────────────────────────────────────────────────────────
#ifdef USE_WIFI
void sntp_sync_rtc(struct timeval* tv) {
  time_t    now = tv->tv_sec;
  struct tm t;
  gmtime_r(&now, &t);
  DateTime dt = {
    (uint16_t)(t.tm_year + 1900), (uint8_t)(t.tm_mon + 1),
    (uint8_t)t.tm_mday, (uint8_t)t.tm_hour,
    (uint8_t)t.tm_min,  (uint8_t)t.tm_sec,
    (uint8_t)t.tm_wday  // 0=Sun, matches PCF8563
  };
  rtc.setDateTime(dt);
  Serial.printf("[ntp] RTC synced → %04d-%02d-%02d %02d:%02d:%02d UTC\n",
                dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
//  splashScreen — boot logo
//
//  Centred text, 5x7 font (each char is 6px wide including 1px gap).
//  x = (128 - numChars * 6) / 2
// ─────────────────────────────────────────────────────────────────────────────
void splashScreen() {
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

// ─────────────────────────────────────────────────────────────────────────────
//  renderDisplay — all OLED output in one place
//
//  Layout (128x64):
//    y=0..13   Row 1 — µS reading
//    y=14..23  Row 2 — HH:MM:SS clock  +  transport status icons (right-aligned)
//    y=24..32  Row 3 — delta / velocity
//    y=43      Divider
//    y=44..63  Polygram strip (20 px)
//
//  Status icons (right side of row 2, 5x7 font):
//    W  WiFi connected      w  WiFi defined but offline
//    B  BLE connected       b  BLE advertising
//    ●  SD recording        -  SD mounted, not recording
//    (nothing shown if the transport is not compiled in)
//
//  µ glyph note:
//    drawStr() treats the string as Latin-1, so the UTF-8 sequence for µ
//    (0xC2 0xB5) renders as two glyphs: Â and µ — giving "ÂµS".
//    drawUTF8() decodes the sequence correctly and renders a single µ.
//    _tf font required (full Latin charset); _tr is ASCII-only.
// ─────────────────────────────────────────────────────────────────────────────
void renderDisplay(float uS, float delta, float delta_c, const DateTime& dt) {
  u8g2.clearBuffer();

  // Row 1 — µS value  (drawUTF8 handles the µ sequence; drawStr would not)
  u8g2.setFont(u8g2_font_7x14B_tf);
  char us_str[20];
  snprintf(us_str, sizeof(us_str), "%.2f µS", uS);
  u8g2.drawUTF8(0, 13, us_str);

  // Row 2 — clock (left) + status icons (right)
  u8g2.setFont(u8g2_font_5x7_tr);
  char time_str[12];
  snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d",
           dt.hour, dt.minute, dt.second);
  u8g2.drawStr(0, 23, time_str);

  // Build status string right-to-left so icons stay flush to the right edge.
  // Each char is 6 px wide (5 px glyph + 1 px gap) in 5x7_tr.
  char status[8] = "";
  int  scol = 0;   // number of chars accumulated

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

  // Reverse so the order reads W B * left-to-right
  for (int i = 0, j = scol - 1; i < j; i++, j--) {
    char tmp = status[i]; status[i] = status[j]; status[j] = tmp;
  }

  int status_x = OLED_W - scol * 6;
  u8g2.drawStr(status_x, 23, status);

  // Row 3 — delta and compressed delta
  u8g2.setFont(u8g2_font_5x7_tf);
  char d_str[32];
  snprintf(d_str, sizeof(d_str), "Δ:%.2f ∂:%.2f", delta, delta_c);
  u8g2.drawUTF8(0, 32, d_str);

  // Bottom strip — 30-second rolling polygram
  draw_polygram();

  u8g2.sendBuffer();
}

// ─────────────────────────────────────────────────────────────────────────────
//  WiFi / WebSocket
// ─────────────────────────────────────────────────────────────────────────────
#ifdef USE_WIFI
  WebsocketsServer wsServer;
  static const int MAX_CLIENTS = 4;
  WebsocketsClient wsClients[MAX_CLIENTS];
  bool             wsConnected[MAX_CLIENTS];

  void ws_init() {
    Serial.print("[wifi] connecting to "); Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
      delay(250); Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\n[wifi] failed — running offline");
      return;
    }
    Serial.print("\n[wifi] IP: "); Serial.println(WiFi.localIP());

    // Register callback then start SNTP — fires automatically in ~1-2s
    sntp_set_time_sync_notification_cb(sntp_sync_rtc);
    configTzTime(TZ_INFO, NTP_SERVER);
    Serial.println("[ntp] SNTP started — waiting for sync...");

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
          Serial.print("[ws]   client connected (slot ");
          Serial.print(i); Serial.println(")");
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
//  BLE (opt-in)
// ─────────────────────────────────────────────────────────────────────────────
#ifdef USE_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
  #define BLE_SERVICE_UUID "454d4554-0000-1000-8000-00805f9b34fb"
  #define BLE_CHAR_UUID    "454d4554-4552-4c45-8d41-52454144494e"   // "EMETER-LE-READIN"
  static BLECharacteristic* g_bleChar      = nullptr;
  static bool               g_bleConnected = false;
  class BLECallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*)    { g_bleConnected = true;  }
    void onDisconnect(BLEServer*) { g_bleConnected = false; }
  };
  void ble_init() {
    BLEDevice::init("emeter");
    BLEServer*  server = BLEDevice::createServer();
    server->setCallbacks(new BLECallbacks());
    BLEService* svc = server->createService(BLE_SERVICE_UUID);
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

  // I2C — one Wire.begin() for the whole bus; OLED and RTC share it
  Wire.begin();

  // RTC — passes already-started Wire; no internal Wire.begin() called
  if (!rtc.begin(Wire)) {
    Serial.println("[rtc] not found — check wiring");
  } else if (!rtc.isRunning()) {
    Serial.println("[rtc] not set — will sync via NTP on WiFi connect");
  } else {
    // Anchor for client-side unix reconstruction:
    //   unix = unix_at_boot + (frame.t - t_ms_at_boot) / 1000
    Serial.printf("[rtc] unix_at_boot=%lu t_ms_at_boot=%lu\n",
                  rtc.getUnixTime(), millis());
  }

  // OLED
  u8g2.begin();
  splashScreen();

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Seed state
  int seed = analogRead(ADC_PIN);
  g_state = { (float)seed, (float)seed, 0.0f, 0, millis() };

  // Warmup — let EMAs settle
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
#ifdef USE_SD
  // SD init after WiFi so RTC is as fresh as possible for the filename
  sd_init(SD_RECORD_ON_BOOT);
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

  String   json = next_frame(g_state);
  float    uS   = adc_to_uS((int)g_state.smoothed);
  uint32_t t_ms = millis() - g_state.t0_ms;

  push_poly(uS);

  // Read RTC once per frame for the clock display only —
  // unix time for data frames is reconstructed client-side from the boot anchor.
  DateTime dt = rtc.getDateTime();
  renderDisplay(uS, g_state.delta, compress(g_state.delta), dt);

  Serial.println(json);
#ifdef USE_WIFI
  ws_send(json);
#endif
#ifdef USE_BLE
  ble_send(json);
#endif
#ifdef USE_SD
  sd_write_frame(t_ms, uS);
#endif

  uint32_t elapsed = micros() - tick;
  if (elapsed < PERIOD_US) delayMicroseconds(PERIOD_US - elapsed);
}
