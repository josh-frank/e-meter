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
 *   • display.h
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

// ── Transport toggles ────────────────────────────────────────────────────────
#define USE_SD
#define USE_WIFI
// #define USE_BLE

#ifdef USE_SD
  #define SD_RECORD_ON_BOOT      true
  #define USE_EXPANSION_BOARD_SD true
#endif

// ── Timezone ─────────────────────────────────────────────────────────────────
static const int TZ_OFFSET = 0;

// ── Includes ──────────────────────────────────────────────────────────────────
#include <Wire.h>
#include <U8g2lib.h>
#include "PCF8563.h"
#include "secrets.h"
#include "display.h"

// ── SD recording toggle ───────────────────────────────────────────────────────
//   Only meaningful when USE_SD is defined.
//   Set true  → recording starts on boot  (like --record flag in index.py)
//   Set false → SD is mounted but recording is off until you add runtime logic
//   Set expansion board → false to use the daughterboard slot
#ifdef USE_SD
  #define SD_RECORD_ON_BOOT      true
  #define USE_EXPANSION_BOARD_SD true
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
  float    uS;       // cached by next_frame — reused by loop()
  float    delta_c;  // cached by next_frame — reused by loop()
  uint32_t count;
  uint32_t t0_ms;
};
static State g_state;

// ── Polygram ring buffer ──────────────────────────────────────────────────────
PolyPoint g_poly[POLY_MAX];
int       g_poly_head = 0;
int       g_poly_len  = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  SD / SRTWriter
// ─────────────────────────────────────────────────────────────────────────────
#ifdef USE_SD
  #include <FS.h>
  #include <SD.h>
  #include <SPI.h>

  #ifdef USE_EXPANSION_BOARD_SD
    static const int SD_CS_PIN = D2;
  #else
    static const int SD_CS_PIN = 21;        // GPIO21 on XIAO ESP32S3 Sense
  #endif
  static const int SD_FLUSH_FRAMES = 20;   // flush every 1 s (20 frames × 50 ms)

  static bool     g_sd_ok      = false;
  bool            g_recording  = false;
  static File     g_srt_file;
  static uint32_t g_srt_index   = 0;
  static int      g_unflushed   = 0;
  static uint32_t g_unix_at_rec = 0;

  // ── ms → SRT timecode  HH:MM:SS,mmm ─────────────────────────────────────
  static void ms_to_tc(char* buf, size_t len, uint32_t ms) {
    uint32_t h = ms / 3600000; ms %= 3600000;
    uint32_t m = ms / 60000;   ms %= 60000;
    uint32_t s = ms / 1000;    ms %= 1000;
    snprintf(buf, len, "%02lu:%02lu:%02lu,%03lu",
             (unsigned long)h, (unsigned long)m,
             (unsigned long)s, (unsigned long)ms);
  }

  // ── Open a new SRT file on the card ──────────────────────────────────────
  static void sd_open_file() {
    if (!g_sd_ok) return;

    if (!SD.exists("/data")) SD.mkdir("/data");

    char path[48];
    DateTime dt = rtc.getDateTime();
    if (dt.year >= 2024) {
      int display_hour = (dt.hour + TZ_OFFSET + 24) % 24;
      snprintf(path, sizeof(path), "/data/eda_%04d%02d%02d_%02d%02d%02d.srt",
               dt.year, dt.month, dt.day, display_hour, dt.minute, dt.second);
    } else {
      snprintf(path, sizeof(path), "/data/eda_%lu.srt", (unsigned long)millis());
    }

    g_srt_file = SD.open(path, "w");
    if (!g_srt_file) {
      Serial.print("[sd] failed to open "); Serial.println(path);
      display_splash_status("sd failed to open!");
      return;
    }

    g_srt_index   = 0;
    g_unflushed   = 0;
    g_unix_at_rec = rtc.isRunning() ? rtc.getUnixTime() : 0;
    g_recording   = true;
    Serial.print("[sd] recording → "); Serial.println(path);
    display_splash_status("sd recording");
  }

  // ── Write one EDA frame as an SRT block ──────────────────────────────────
  static void sd_write_frame(uint32_t t_ms, float uS) {
    if (!g_recording || !g_srt_file) return;

    g_srt_index++;

    char tc_start[16], tc_end[16];
    ms_to_tc(tc_start, sizeof(tc_start), t_ms);
    ms_to_tc(tc_end,   sizeof(tc_end),   t_ms + (uint32_t)(PERIOD_US / 1000));

    char block[128];
    int n;
    if (g_unix_at_rec > 0) {
      float unix_f = (float)g_unix_at_rec + (float)t_ms / 1000.0f;
      n = snprintf(block, sizeof(block),
                   "%lu\n%s --> %s\n%.3f | %.4f\n\n",
                   (unsigned long)g_srt_index,
                   tc_start, tc_end, unix_f, uS);
    } else {
      n = snprintf(block, sizeof(block),
                   "%lu\n%s --> %s\n%.4f\n\n",
                   (unsigned long)g_srt_index,
                   tc_start, tc_end, uS);
    }
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
    #ifdef USE_EXPANSION_BOARD_SD
      SPI.begin(D8, D9, D10, D2);
    #endif
    if (!SD.begin(SD_CS_PIN)) {
      Serial.println("[sd] mount failed — check card (FAT32, ≤32 GB)");
      display_splash_status("sd mount failed!");
      return;
    }
    g_sd_ok = true;
    Serial.printf("[sd] card mounted  (%.1f MB free)\n",
                  (float)(SD.totalBytes() - SD.usedBytes()) / 1048576.0f);
    display_splash_status("sd card mounted");
    if (record_on_boot) sd_open_file();
  }
#endif  // USE_SD

// ─────────────────────────────────────────────────────────────────────────────
//  WiFi / WebSocket
// ─────────────────────────────────────────────────────────────────────────────
#ifdef USE_WIFI
  #include <WiFi.h>
  #include <ArduinoWebsockets.h>
  #include <esp_sntp.h>
  using namespace websockets;

  const char*    WIFI_SSID  = SECRET_SSID;
  const char*    WIFI_PASS  = SECRET_PASS;
  const uint16_t WS_PORT    = SECRET_PORT;
  const char*    NTP_SERVER = "pool.ntp.org";

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

    sntp_set_time_sync_notification_cb(sntp_sync_rtc);
    configTzTime("UTC0", NTP_SERVER);
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
#endif  // USE_WIFI

// ─────────────────────────────────────────────────────────────────────────────
//  BLE (opt-in)
// ─────────────────────────────────────────────────────────────────────────────
#ifdef USE_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>
  #define BLE_SERVICE_UUID "454d4554-0000-1000-8000-00805f9b34fb"
  #define BLE_CHAR_UUID    "454d4554-4552-4c45-8d41-52454144494e"
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
#endif  // USE_BLE

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
  s.delta_c      = compress(s.delta);
  float vel_c    = compress(velocity);

  s.count++;

  uint32_t t_ms = millis() - s.t0_ms;
  s.uS          = adc_to_uS((int)s.smoothed);

  char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"t\":%lu,\"smooth_uS\":%.2f,\"delta\":%.3f,\"delta_c\":%.3f,\"velocity\":%.3f}",
    (unsigned long)t_ms, s.uS, s.delta, s.delta_c, vel_c);
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
//  sntp_sync_rtc
//  Called automatically by the ESP32 SNTP stack once time is confirmed.
//  Pushes synced UTC time into the PCF8563 hardware RTC.
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
  Serial.printf("[ntp] RTC synced → %04d-%02d-%02d %02d:%02d:%02d\n",
                dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
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

  // RTC
  if (!rtc.begin(Wire)) {
    Serial.println("[rtc] not found — check wiring");
  } else if (!rtc.isRunning()) {
    Serial.println("[rtc] not set — will sync via NTP on WiFi connect");
  } else {
    Serial.printf("[rtc] unix_at_boot=%lu t_ms_at_boot=%lu\n",
                  rtc.getUnixTime(), millis());
  }

  // OLED + display
  u8g2.begin();
  display_splash();   // boot logo — defined in display.h
  display_setup();    // wires up button ISR

  // ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Seed state
  int seed = analogRead(ADC_PIN);
  g_state = { (float)seed, (float)seed, 0.0f, 0.0f, 0.0f, 0, millis() };

  // Warmup — let EMAs settle
  Serial.println("[emeter] warming up...");
  display_splash_status("warming up...");
  for (int i = 0; i < WARMUP_SAMP; i++) {
    next_frame(g_state);
    delayMicroseconds(PERIOD_US);
  }
  g_state.count = 0;
  g_state.t0_ms = millis();
  Serial.println("[emeter] warmup done");
  display_splash_status("warmup done");

#ifdef USE_WIFI
  ws_init();
#endif
#ifdef USE_BLE
  ble_init();
#endif
#ifdef USE_SD
  sd_init(SD_RECORD_ON_BOOT);
#endif

  Serial.println("[emeter] streaming at 20 Hz");
  display_splash_status("streaming at 20 Hz");
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
  uint32_t t_ms = millis() - g_state.t0_ms;

  push_poly(g_state.uS);

  // RTC read once per frame for the clock display only.
  // Unix time for data frames is reconstructed client-side from the boot anchor.
  DateTime dt = rtc.getDateTime();
  display_render(g_state.uS, g_state.delta, g_state.delta_c, dt);

#ifdef USE_WIFI
  ws_send(json);
#endif
#ifdef USE_BLE
  ble_send(json);
#endif
#ifdef USE_SD
  sd_write_frame(t_ms, g_state.uS);
#endif

  uint32_t elapsed = micros() - tick;
  if (elapsed < PERIOD_US) delayMicroseconds(PERIOD_US - elapsed);
}
