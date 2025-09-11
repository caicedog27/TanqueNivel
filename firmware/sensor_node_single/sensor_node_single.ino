#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// -----------------------------------------------------------------------------
// Generic firmware for a single ultrasonic sensor board (ESP32)
// Each board reads one DYP-A01 sensor and reports distance/level
// to the master board via ESP-NOW.
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

// MAC address of the master (gateway) board
uint8_t MASTER_MAC[] = {0xA0, 0xB7, 0x65, 0x28, 0x71, 0x3C};

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

// Forward declarations
void sendSensorData();
uint16_t readDYP(HardwareSerial &port);
float distToPercent(uint16_t dist);
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);
void loadConfig();
void saveConfig();

// Serial instance for sensor
HardwareSerial sensorSerial(SENSOR_UART_NUM);

void setup() {
  Serial.begin(115200);
  loadConfig();

  // Start sensor UART
  sensorSerial.begin(9600, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);

  // Add master peer
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, MASTER_MAC, 6);
  peer.channel = 0; // same channel
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void loop() {
  static unsigned long lastSend = 0;
  unsigned long now = millis();
  if (now - lastSend >= 500) {
    lastSend = now;
    sendSensorData();
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

  char buffer[256];
  size_t len = serializeJson(doc, buffer);
  if (esp_now_send(MASTER_MAC, (uint8_t*)buffer, len) != ESP_OK) {
    Serial.println("ESP-NOW send error");
  }
}

// Read the latest distance frame from the DYP-A01 sensor.
// The sensor refreshes its measurement roughly every 100 ms. To get a
// reliable value we drain all available frames and keep the most recent
// valid one, waiting briefly if no fresh data has arrived yet.
uint16_t readDYP(HardwareSerial &port) {
  uint16_t dist = 0;
  unsigned long start = millis();

  // allow up to ~120ms (one measurement period plus margin) for data
  while (millis() - start < 120) {
    while (port.available() >= 4) {
      int header = port.read();
      if (header != 0xFF) {
        continue;  // not a valid frame header
      }
      int high = port.read();
      int low  = port.read();
      int sum  = port.read();
      if (((0xFF + high + low) & 0xFF) == sum) {
        dist = ((uint16_t)high << 8) | low;  // store latest valid reading
      }
    }
    if (dist) {
      break;  // got a measurement
    }
    delay(5);  // wait a bit for the sensor to produce a frame
  }

  // Flush leftover bytes so the next call starts fresh
  while (port.available()) {
    port.read();
  }

  return dist;
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

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (!incomingData || len <= 0) return;
  String msg((const char*)incomingData, len);
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, msg) != DeserializationError::Ok) {
    Serial.println("JSON parse error");
    return;
  }
  const char* type = doc["type"];
  if (!type) return;
  if (strcmp(type, "config_set") == 0) {
    JsonObject payload = doc["payload"];
    if (payload.containsKey("empty_mm")) cfg.empty_mm = payload["empty_mm"].as<float>();
    if (payload.containsKey("full_mm")) cfg.full_mm = payload["full_mm"].as<float>();
    // Backwards compatibility for legacy keys
    if (payload.containsKey("empty_tanque_mm")) cfg.empty_mm = payload["empty_tanque_mm"].as<float>();
    if (payload.containsKey("full_tanque_mm")) cfg.full_mm = payload["full_tanque_mm"].as<float>();
    cfgUpdated = true;
  } else if (strcmp(type, "ping") == 0) {
    StaticJsonDocument<128> out;
    out["type"] = "hb";
    out["from"] = SENSOR_ID;
    char buffer[128];
    size_t lenOut = serializeJson(out, buffer);
    esp_now_send(MASTER_MAC, (uint8_t*)buffer, lenOut);
  }
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
