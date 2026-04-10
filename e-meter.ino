/**
 *    ◢◣
 *   ◢■■◣
 *  ◢■■■■◣
 * emeter.ino  —  XIAO ESP32C3 electropsychometer
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
 * Libraries required (install via Arduino Library Manager):
 *   • ArduinoWebsockets  by Gil Maimon
 *   • ArduinoJson        by Benoît Blanchon
 *   (BLE) — built-in ESP32 BLE library, no extra install needed
 *
 * Board: "XIAO_ESP32C3" in Arduino IDE
 *        (Boards Manager → esp32 by Espressif)
 */

// Transport toggles
#define USE_WIFI        // WebSocket server on WiFi (comment out to disable)
// #define USE_BLE      // BLE notify (uncomment to enable)

// WiFi credentials (ignored if USE_WIFI is not defined)
#ifdef USE_WIFI
  const char* WIFI_SSID = "E-METER";
  const char* WIFI_PASS = "trueclear";
  const uint16_t WS_PORT = 5002;
#endif

// ADC & sensor constants
static const int    ADC_PIN       = A0;
static const float  GSR_RREF     = 100000.0f;   // 100 kΩ reference
static const float  GSR_VSUPPLY  = 3.3f;
static const float  ADC_MAX      = 4095.0f;     // 12-bit on ESP32
static const int    ADC_CLAMP_LO = 1;
static const int    ADC_CLAMP_HI = 4094;        // scaled from 1022/1023

// EMA & timing
static const float  EMA_FAST     = 0.15f;
static const float  EMA_SLOW     = 0.005f;
static const int    WARMUP_SAMP  = 100;
static const uint32_t PERIOD_US  = 50000;       // 20 Hz  =  50 000 µs

// State
struct State {
  float    smoothed;
  float    baseline;
  float    delta;
  uint32_t count;
  uint32_t t0_ms;
};
static State g_state;

// Forward declarations────
float  adc_to_uS(int raw);
float  compress(float x);
String next_frame(State& s);

// WiFi / WebSocket─
#ifdef USE_WIFI
  #include <WiFi.h>
  #include <ArduinoWebsockets.h>
  using namespace websockets;

  WebsocketsServer wsServer;
  // hold up to 4 simultaneous clients (browser tabs)
  static const int MAX_CLIENTS = 4;
  WebsocketsClient wsClients[MAX_CLIENTS];
  bool             wsConnected[MAX_CLIENTS];

  void ws_init() {
    Serial.print("[wifi] connecting to ");
    Serial.println(WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();
    Serial.print("[wifi] IP: ");
    Serial.println(WiFi.localIP());
    wsServer.listen(WS_PORT);
    Serial.print("[ws]   listening on port ");
    Serial.println(WS_PORT);
    memset(wsConnected, false, sizeof(wsConnected));
  }

  void ws_accept() {
    if (wsServer.poll()) {
      WebsocketsClient client = wsServer.accept();
      for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!wsConnected[i]) {
          wsClients[i]    = client;
          wsConnected[i]  = true;
          Serial.print("[ws]   client connected (slot ");
          Serial.print(i);
          Serial.println(")");
          break;
        }
      }
    }
  }

  void ws_send(const String& json) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
      if (wsConnected[i]) {
        if (wsClients[i].available()) {
          wsClients[i].send(json);
        } else {
          wsConnected[i] = false;   // client disconnected
          Serial.print("[ws]   client disconnected (slot ");
          Serial.print(i);
          Serial.println(")");
        }
      }
    }
  }
#endif   // USE_WIFI

// BLE
#ifdef USE_BLE
  #include <BLEDevice.h>
  #include <BLEServer.h>
  #include <BLEUtils.h>
  #include <BLE2902.h>

  #define BLE_SERVICE_UUID  "12345678-1234-1234-1234-123456789abc"
  #define BLE_CHAR_UUID     "abcdefab-cdef-abcd-efab-cdefabcdefab"

  static BLECharacteristic* g_bleChar = nullptr;
  static bool g_bleConnected = false;

  class BLECallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*)    { g_bleConnected = true;  Serial.println("[ble]  client connected"); }
    void onDisconnect(BLEServer*) { g_bleConnected = false; Serial.println("[ble]  client disconnected"); }
  };

  void ble_init() {
    BLEDevice::init("emeter");
    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(new BLECallbacks());
    BLEService* svc   = server->createService(BLE_SERVICE_UUID);
    g_bleChar = svc->createCharacteristic(
      BLE_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
    );
    g_bleChar->addDescriptor(new BLE2902());
    svc->start();
    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SERVICE_UUID);
    adv->start();
    Serial.println("[ble]  advertising as 'emeter'");
  }

  void ble_send(const String& json) {
    if (!g_bleConnected) return;
    g_bleChar->setValue((uint8_t*)json.c_str(), json.length());
    g_bleChar->notify();
  }
#endif   // USE_BLE

//  Note: clamp limits are scaled to 12-bit (×4 from original 10-bit)
float adc_to_uS(int raw) {
  raw = max(ADC_CLAMP_LO, min(ADC_CLAMP_HI, raw));
  float v = (raw / ADC_MAX) * GSR_VSUPPLY;
  float r = GSR_RREF * ((GSR_VSUPPLY / v) - 1.0f);
  return (1.0f / r) * 1e6f;
}

float compress(float x) {
  return x / (1.0f + fabsf(x) * 0.05f);
}

//  Returns a JSON string matching the exact schema index.html expects
String next_frame(State& s) {
  int raw = analogRead(ADC_PIN);

  // EMA fast (smoothing)
  s.smoothed = EMA_FAST * raw + (1.0f - EMA_FAST) * s.smoothed;

  // EMA slow (baseline drift)
  s.baseline = EMA_SLOW * s.smoothed + (1.0f - EMA_SLOW) * s.baseline;

  float prev_delta = s.delta;

  float raw_delta = s.smoothed - s.baseline;
  float norm      = max(10.0f, fabsf(s.baseline));
  s.delta         = (raw_delta / norm) * 100.0f;

  float velocity  = s.delta - prev_delta;

  float delta_c   = compress(s.delta);
  float vel_c     = compress(velocity);

  s.count++;

  uint32_t t_ms   = millis() - s.t0_ms;
  float    uS     = adc_to_uS((int)s.smoothed);

  // Matches the exact keys index.html reads (smooth_uS, delta,
  // delta_c, velocity) so the frontend works with zero changes.
  char buf[128];
  snprintf(buf, sizeof(buf),
    "{\"t\":%lu,\"smooth_uS\":%.2f,\"delta\":%.3f,\"delta_c\":%.3f,\"velocity\":%.3f}",
    (unsigned long)t_ms,
    uS,
    s.delta,
    delta_c,
    vel_c
  );
  return String(buf);
}

//  setup
void setup() {
  Serial.begin(115200);
  delay(500);   // let USB-serial enumerate
  Serial.println("[emeter] booting");

  // 12-bit ADC on ESP32 (matches ADC_MAX = 4095)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);   // full 0–3.3V range

  // Seed state from a real ADC read
  int seed = analogRead(ADC_PIN);
  g_state.smoothed = (float)seed;
  g_state.baseline = (float)seed;
  g_state.delta    = 0.0f;
  g_state.count    = 0;
  g_state.t0_ms    = millis();

  // Warmup — run WARMUP_SAMP samples silently to let EMAs settle
  Serial.println("[emeter] warming up...");
  for (int i = 0; i < WARMUP_SAMP; i++) {
    next_frame(g_state);
    delayMicroseconds(PERIOD_US);
  }
  g_state.count = 0;   // reset counter after warmup
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

//  loop  —  20 Hz fixed-rate loop
void loop() {
  uint32_t tick = micros();

#ifdef USE_WIFI
  ws_accept();   // accept any pending new connections each frame
#endif

  String json = next_frame(g_state);

  // Serial — always on
  Serial.println(json);

#ifdef USE_WIFI
  ws_send(json);
#endif

#ifdef USE_BLE
  ble_send(json);
#endif

  // Fixed-rate sleep — mirrors: await asyncio.sleep(max(0, SAMPLE_S - elapsed))
  uint32_t elapsed = micros() - tick;
  if (elapsed < PERIOD_US) {
    delayMicroseconds(PERIOD_US - elapsed);
  }
}
