#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// Configure UARTs for DYP-A01 sensors
HardwareSerial dypTanque(2);
HardwareSerial dypTolva(1);

#define TANQUE_RX_PIN 16
#define TANQUE_TX_PIN 17
#define TOLVA_RX_PIN   4
#define TOLVA_TX_PIN   2

// Address of the master (gateway) board. Replace with your master's MAC
uint8_t MASTER_MAC[] = {0xA0, 0xB7, 0x65, 0x28, 0x71, 0x3C};

// Structure to hold calibration parameters for sensors
struct Config {
  // Distances in millimetres measured when tanks are empty and full.
  // These values are used to map raw distance readings to percentage.  When
  // the tank is empty, the measured distance from the sensor to the liquid
  // surface equals `empty_*`.  When the tank is full, the measured distance
  // equals `full_*`.  The conversion to percentage is:
  //   pct = ((emptyDist - dist) / (emptyDist - fullDist)) * 100
  float empty_tanque;
  float full_tanque;
  float empty_tolva;
  float full_tolva;
} cfg;

// Preferences for storing calibration values in NVS
Preferences prefs;

// Sequence number for packets
static uint32_t seq_num = 0;

// Circular buffers for median filtering (store last 5 readings)
#define BUF_LEN 5
uint16_t bufTanque[BUF_LEN] = {0};
uint16_t bufTolva[BUF_LEN]  = {0};
uint8_t bufIndex = 0;

// Flag set when a configuration update is received
volatile bool cfgUpdated = false;

// Forward declarations
void sendSensorData();
uint16_t readDYP(HardwareSerial &port);
float distToPercent(uint16_t dist, float emptyDist, float fullDist);
// esp_now_recv_cb_t changed in ESP-IDF v5.0+.  The callback now receives a
// pointer to esp_now_recv_info_t that contains the source MAC and other
// information.  See: docs.espressif.com/en/latest/api-reference/wifi/esp_now.html
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);
void loadConfig();
void saveConfig();

void setup() {
  Serial.begin(115200);
  // Initialize preferences
  loadConfig();

  // Start sensors
  dypTanque.begin(9600, SERIAL_8N1, TANQUE_RX_PIN, TANQUE_TX_PIN);
  dypTolva.begin(9600, SERIAL_8N1, TOLVA_RX_PIN, TOLVA_TX_PIN);

  // Configure WiFi for ESP-NOW (station mode)
  WiFi.mode(WIFI_STA);
  // Channel must match the master. We assume master sets channel after connecting to AP.
  // If known, set the channel here for more reliable connection.

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
  }
  // Register the receive callback.  Note: the signature expects
  // esp_now_recv_info_t* as first argument starting with ESP-IDF v5.x.
  esp_now_register_recv_cb(onDataRecv);

  // Add master as peer
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, MASTER_MAC, 6);
  peer.channel = 0;  // 0 matches current WiFi channel
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add master peer");
  }
}

void loop() {
  static unsigned long lastSend = 0;
  unsigned long now = millis();
  // send data every 500 ms
  if (now - lastSend >= 500) {
    lastSend = now;
    sendSensorData();
  }
  // If configuration updated, save to NVS
  if (cfgUpdated) {
    cfgUpdated = false;
    saveConfig();
  }
}

// Send sensor readings to master as JSON via ESP-NOW
void sendSensorData() {
  // Read sensors
  uint16_t dTanque = readDYP(dypTanque);
  uint16_t dTolva  = readDYP(dypTolva);

  // Store in circular buffer for median filtering
  bufTanque[bufIndex] = dTanque;
  bufTolva[bufIndex]  = dTolva;
  bufIndex = (bufIndex + 1) % BUF_LEN;

  // Copy buffers to temporary arrays for sorting
  uint16_t tempTanque[BUF_LEN];
  uint16_t tempTolva[BUF_LEN];
  memcpy(tempTanque, bufTanque, sizeof(bufTanque));
  memcpy(tempTolva,  bufTolva,  sizeof(bufTolva));

  // Simple insertion sort for median
  for (int i = 1; i < BUF_LEN; i++) {
    uint16_t key = tempTanque[i];
    int j = i - 1;
    while (j >= 0 && tempTanque[j] > key) {
      tempTanque[j + 1] = tempTanque[j];
      j--;
    }
    tempTanque[j + 1] = key;
  }
  for (int i = 1; i < BUF_LEN; i++) {
    uint16_t key = tempTolva[i];
    int j = i - 1;
    while (j >= 0 && tempTolva[j] > key) {
      tempTolva[j + 1] = tempTolva[j];
      j--;
    }
    tempTolva[j + 1] = key;
  }
  // Median values
  uint16_t mTanque = tempTanque[BUF_LEN / 2];
  uint16_t mTolva  = tempTolva[BUF_LEN / 2];

  // Convert to percentage using calibration
  float pctTanque = distToPercent(mTanque, cfg.empty_tanque, cfg.full_tanque);
  float pctTolva  = distToPercent(mTolva,  cfg.empty_tolva,  cfg.full_tolva);

  // Build JSON document
  StaticJsonDocument<256> doc;
  doc["type"] = "sensor_data";
  doc["src"]  = "SENS";
  doc["seq"]  = ++seq_num;
  JsonArray items = doc.createNestedArray("items");
  JsonObject tItem = items.createNestedObject();
  tItem["id"] = "TANQUE";
  tItem["dist_mm"]   = mTanque;
  tItem["level_pct"] = isnan(pctTanque) ? -1.0 : pctTanque;
  JsonObject lItem = items.createNestedObject();
  lItem["id"] = "TOLVA";
  lItem["dist_mm"]   = mTolva;
  lItem["level_pct"] = isnan(pctTolva) ? -1.0 : pctTolva;

  // Serialize to string
  char buffer[256];
  size_t len = serializeJson(doc, buffer);

  // Send to master
  esp_err_t result = esp_now_send(MASTER_MAC, (uint8_t*)buffer, len);
  if (result != ESP_OK) {
    Serial.print("ESP-NOW send error: ");
    Serial.println(result);
  }
}

// Read 4-byte frame from DYP-A01 sensor (returns 0 if invalid)
uint16_t readDYP(HardwareSerial &port) {
  while (port.available() >= 4) {
    int header = port.read();
    if (header == 0xFF) {
      int high = port.read();
      int low  = port.read();
      int sum  = port.read();
      if (((0xFF + high + low) & 0xFF) == sum) {
        uint16_t dist = ((uint16_t)high << 8) | low;
        return dist;
      }
    }
  }
  return 0;
}

// Convert raw distance to percentage (0-100%). Returns NAN if invalid.
float distToPercent(uint16_t dist, float emptyDist, float fullDist) {
  if (dist == 0) return NAN;
  float range = emptyDist - fullDist;
  if (range <= 0.0f) return NAN;
  float pct = ((emptyDist - (float)dist) / range) * 100.0f;
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  return pct;
}

// Callback to handle incoming configuration messages or pings.
// The new esp_now_recv_cb_t signature passes a pointer to esp_now_recv_info_t
// containing the sender MAC and other metadata.  We ignore the info here.
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (!incomingData || len <= 0) return;
  // Copy incoming data to a String for JSON parsing
  String msg((const char*)incomingData, len);
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }
  const char* type = doc["type"];
  if (!type) return;
  // Handle configuration update: update calibration distances
  if (strcmp(type, "config_set") == 0) {
    JsonObject payload = doc["payload"];
    // For backwards compatibility, support old keys (H_tanque_mm, etc.)
    if (payload.containsKey("empty_tanque_mm")) cfg.empty_tanque = payload["empty_tanque_mm"].as<float>();
    if (payload.containsKey("full_tanque_mm"))  cfg.full_tanque  = payload["full_tanque_mm"].as<float>();
    if (payload.containsKey("empty_tolva_mm"))  cfg.empty_tolva  = payload["empty_tolva_mm"].as<float>();
    if (payload.containsKey("full_tolva_mm"))   cfg.full_tolva   = payload["full_tolva_mm"].as<float>();
    // Also support legacy keys if provided
    if (payload.containsKey("H_tanque_mm")) cfg.empty_tanque = payload["H_tanque_mm"].as<float>();
    if (payload.containsKey("H_tolva_mm"))  cfg.empty_tolva  = payload["H_tolva_mm"].as<float>();
    // if dead_zone_mm provided, ignore for now (no dead zone used)
    cfgUpdated = true;
    Serial.println("Sensor calibration updated remotely");
  }
  else if (strcmp(type, "ping") == 0) {
    // Respond to ping with heartbeat
    StaticJsonDocument<128> docOut;
    docOut["type"] = "hb";
    docOut["from"] = "SENS";
    char buffer[128];
    size_t lenOut = serializeJson(docOut, buffer);
    esp_now_send(MASTER_MAC, (uint8_t*)buffer, lenOut);
  }
}

void loadConfig() {
  prefs.begin("sensors", true);
  cfg.empty_tanque = prefs.getFloat("empty_tanque", 1200.0f);
  cfg.full_tanque  = prefs.getFloat("full_tanque", 200.0f);
  cfg.empty_tolva  = prefs.getFloat("empty_tolva", 800.0f);
  cfg.full_tolva   = prefs.getFloat("full_tolva", 100.0f);
  prefs.end();
  Serial.printf("Loaded config: empty_tanque=%.1f mm, full_tanque=%.1f mm, empty_tolva=%.1f mm, full_tolva=%.1f mm\n",
                cfg.empty_tanque, cfg.full_tanque, cfg.empty_tolva, cfg.full_tolva);
}

void saveConfig() {
  prefs.begin("sensors", false);
  prefs.putFloat("empty_tanque", cfg.empty_tanque);
  prefs.putFloat("full_tanque",  cfg.full_tanque);
  prefs.putFloat("empty_tolva",  cfg.empty_tolva);
  prefs.putFloat("full_tolva",   cfg.full_tolva);
  prefs.end();
  Serial.println("Calibration saved to NVS");
}

