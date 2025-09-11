#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// -----------------------------------------------------------------------------
// Generic firmware for a single ultrasonic sensor board (ESP32)
// Each board reads one DYP-A01 sensor and reports distance/level
// directly to the Python server via WiFi/WebSocket.
// -----------------------------------------------------------------------------

// ---- Board configuration ----------------------------------------------------
// Identifier of this sensor. Change for each unit (e.g. "TANQUE", "TOLVA")
#ifndef SENSOR_ID
#define SENSOR_ID "SENSOR"
#endif

// UART configuration for the DYP-A01 sensor
#ifndef SENSOR_UART_NUM
#define SENSOR_UART_NUM 1
#endif
#ifndef SENSOR_RX_PIN
#define SENSOR_RX_PIN 16
#endif
#ifndef SENSOR_TX_PIN
#define SENSOR_TX_PIN 17
#endif
#ifndef SENSOR_TRIG_PIN
#define SENSOR_TRIG_PIN 4
#endif

// WiFi credentials and server endpoint
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";
const char* WS_HOST  = "192.168.1.100"; // change to server IP
const uint16_t WS_PORT = 8000;
const char* WS_PATH = "/ws/board/SENS"; // WebSocket endpoint for this board

// Preferences storage for calibration parameters
Preferences prefs;

// Calibration parameters
struct Config {
  float empty_mm;  // measured distance when container is empty
  float full_mm;   // measured distance when container is full
} cfg;

// Sequence number for packets
static uint32_t seq_num = 0;

// Flag when configuration is updated
volatile bool cfgUpdated = false;

// WebSocket client and heartbeat timing
WebSocketsClient ws;
const unsigned long HB_INTERVAL_MS = 2000;
unsigned long lastHb = 0;

// Forward declarations
struct Reading { uint16_t mm; bool ok; };
void triggerOnce();
Reading readFrame(HardwareSerial &port, uint32_t timeout_ms = 80);
uint16_t readDYP(HardwareSerial &port, uint8_t samples = 5, uint16_t maxStep = 400);
void sendSensorData();
float distToPercent(uint16_t dist);
void loadConfig();
void saveConfig();
void sendHeartbeat();
void sendToServer(const JsonDocument& doc);
void onWsEvent(WStype_t type, uint8_t *payload, size_t length);

// Serial instance for sensor
HardwareSerial sensorSerial(SENSOR_UART_NUM);

void setup() {
  Serial.begin(115200);
  loadConfig();

  // Start sensor UART
  sensorSerial.begin(9600, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
  pinMode(SENSOR_TRIG_PIN, OUTPUT);
  digitalWrite(SENSOR_TRIG_PIN, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.println(" connected");

  ws.begin(WS_HOST, WS_PORT, WS_PATH);
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(5000);
}

void loop() {
  static unsigned long lastSend = 0;
  unsigned long now = millis();
  ws.loop();
  if (now - lastSend >= 500) {
    lastSend = now;
    sendSensorData();
  }
  if (now - lastHb >= HB_INTERVAL_MS) {
    lastHb = now;
    sendHeartbeat();
  }
  if (cfgUpdated) {
    cfgUpdated = false;
    saveConfig();
  }
}

void sendSensorData() {
  uint16_t dist = readDYP(sensorSerial);
  float pct = distToPercent(dist);

  StaticJsonDocument<256> doc;
  doc["type"] = "sensor_data";
  // keep source identifier as generic sensor board so that the existing
  // server logic can aggregate multiple sensors
  doc["src"] = "SENS";
  doc["seq"] = ++seq_num;
  JsonArray items = doc.createNestedArray("items");
  JsonObject item = items.createNestedObject();
  item["id"] = SENSOR_ID;
  item["dist_mm"] = dist;
  item["level_pct"] = isnan(pct) ? -1.0 : pct;

  sendToServer(doc);
}

// Trigger a single ultrasonic measurement on the DYP-A01 sensor (controlled mode)
void triggerOnce() {
  digitalWrite(SENSOR_TRIG_PIN, HIGH);
  delayMicroseconds(200);
  digitalWrite(SENSOR_TRIG_PIN, LOW);
  delayMicroseconds(2000);
  digitalWrite(SENSOR_TRIG_PIN, HIGH);
}

// Read one UART frame from the sensor. Returns {mm,ok}.
Reading readFrame(HardwareSerial &port, uint32_t timeout_ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (port.available()) {
      uint8_t b = port.read();
      if (b == 0xFF) {
        while (port.available() < 3) {
          if (millis() - t0 > timeout_ms) break;
        }
        if (port.available() >= 3) {
          uint8_t dh = port.read();
          uint8_t dl = port.read();
          uint8_t sum = port.read();
          uint8_t calc = (0xFF + dh + dl) & 0xFF;
          uint16_t mm = ((uint16_t)dh << 8) | dl;
          bool ok = (calc == sum) && (mm >= 280 && mm <= 7500);
          return {mm, ok};
        }
      }
    }
  }
  return {0, false};
}

// Robust distance read using median and step rejection
uint16_t readDYP(HardwareSerial &port, uint8_t samples, uint16_t maxStep) {
  static uint16_t last = 0;
  uint16_t buf[9];
  uint8_t k = 0;
  for (uint8_t i = 0; i < samples; ++i) {
    triggerOnce();
    delay(60);  // sensor processing time
    Reading r = readFrame(port);
    if (r.ok) buf[k++] = r.mm;
    delay(15);
  }
  if (!k) return 0;

  for (uint8_t i = 0; i < k; ++i) {
    for (uint8_t j = i + 1; j < k; ++j) {
      if (buf[j] < buf[i]) {
        uint16_t tmp = buf[i];
        buf[i] = buf[j];
        buf[j] = tmp;
      }
    }
  }
  uint16_t med = buf[k / 2];
  if (last && abs((int)med - (int)last) > maxStep) {
    med = last;
  }
  last = med;
  return med;
}

float distToPercent(uint16_t dist) {
  if (dist == 0) return NAN;
  float range = cfg.empty_mm - cfg.full_mm;
  if (range <= 0.0f) return NAN;
  float pct = ((cfg.empty_mm - (float)dist) / range) * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

void onWsEvent(WStype_t type, uint8_t *payload, size_t length) {
  if (type != WStype_TEXT) return;
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.println("WS parse error");
    return;
  }
  const char* mtype = doc["type"];
  if (!mtype) return;
  if (strcmp(mtype, "config_set") == 0) {
    JsonObject pl = doc["payload"];
    if (pl.containsKey("empty_mm")) cfg.empty_mm = pl["empty_mm"].as<float>();
    if (pl.containsKey("full_mm")) cfg.full_mm = pl["full_mm"].as<float>();
    if (pl.containsKey("empty_tanque_mm")) cfg.empty_mm = pl["empty_tanque_mm"].as<float>();
    if (pl.containsKey("full_tanque_mm")) cfg.full_mm = pl["full_tanque_mm"].as<float>();
    cfgUpdated = true;
  }
}

void sendHeartbeat() {
  StaticJsonDocument<128> doc;
  doc["type"] = "hb";
  doc["from"] = "SENS";
  sendToServer(doc);
}

void sendToServer(const JsonDocument& doc) {
  char buffer[256];
  size_t len = serializeJson(doc, buffer);
  ws.sendTXT(buffer, len);
}

void loadConfig() {
  prefs.begin("sens", true);
  cfg.empty_mm = prefs.getFloat("empty", 1200.0f);
  cfg.full_mm  = prefs.getFloat("full", 200.0f);
  prefs.end();
  Serial.printf("Loaded config: empty=%.1fmm full=%.1fmm\n", cfg.empty_mm, cfg.full_mm);
}

void saveConfig() {
  prefs.begin("sens", false);
  prefs.putFloat("empty", cfg.empty_mm);
  prefs.putFloat("full", cfg.full_mm);
  prefs.end();
  Serial.println("Calibration saved to NVS");
}
