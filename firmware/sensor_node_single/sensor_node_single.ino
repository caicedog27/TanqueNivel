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

// Median filter buffer
static const uint8_t BUF_LEN = 5;
uint16_t buf[BUF_LEN] = {0};
uint8_t bufIndex = 0;

// Sequence number for packets
static uint32_t seq_num = 0;

// Flag when configuration is updated
volatile bool cfgUpdated = false;

// WebSocket client and heartbeat timing
WebSocketsClient ws;
const unsigned long HB_INTERVAL_MS = 2000;
unsigned long lastHb = 0;

// Forward declarations
void sendSensorData();
uint16_t readDYP(HardwareSerial &port);
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
  buf[bufIndex] = dist;
  bufIndex = (bufIndex + 1) % BUF_LEN;

  // copy buffer for sorting
  uint16_t temp[BUF_LEN];
  memcpy(temp, buf, sizeof(buf));
  for (int i = 1; i < BUF_LEN; ++i) {
    uint16_t key = temp[i];
    int j = i - 1;
    while (j >= 0 && temp[j] > key) {
      temp[j + 1] = temp[j];
      j--;
    }
    temp[j + 1] = key;
  }
  uint16_t median = temp[BUF_LEN / 2];
  float pct = distToPercent(median);

  StaticJsonDocument<256> doc;
  doc["type"] = "sensor_data";
  // keep source identifier as generic sensor board so that the existing
  // server logic can aggregate multiple sensors
  doc["src"] = "SENS";
  doc["seq"] = ++seq_num;
  JsonArray items = doc.createNestedArray("items");
  JsonObject item = items.createNestedObject();
  item["id"] = SENSOR_ID;
  item["dist_mm"] = median;
  item["level_pct"] = isnan(pct) ? -1.0 : pct;

  sendToServer(doc);
}

uint16_t readDYP(HardwareSerial &port) {
  while (port.available() >= 4) {
    int header = port.read();
    if (header == 0xFF) {
      int high = port.read();
      int low = port.read();
      int sum = port.read();
      if (((0xFF + high + low) & 0xFF) == sum) {
        return ((uint16_t)high << 8) | low;
      }
    }
  }
  return 0;
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
