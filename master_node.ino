#include <WiFi.h>
#include <WebSocketsClient.h>
#include <esp_now.h>
#include <ArduinoJson.h>

// WiFi credentials (use 2.4 GHz network)
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";

// WebSocket server (Python server) configuration
const char* WS_HOST = "192.168.1.100"; // update with server IP
const uint16_t WS_PORT = 8000;
const char* WS_PATH = "/ws/bridge";

// MAC addresses of other boards
uint8_t SENSOR_MAC[]   = {0xA0, 0xB7, 0x65, 0x2D, 0xC5, 0xD4}; // sensors node
uint8_t ACTUATOR_MAC[] = {0xC4, 0x4F, 0x33, 0x79, 0xB3, 0x45}; // actuator (relay) node

// WebSocket client
WebSocketsClient ws;

// Last timestamps for sensor and actuator heartbeats
unsigned long lastSensorHB = 0;
unsigned long lastActHB    = 0;

// Timeout thresholds (ms)
const unsigned long SENSOR_TIMEOUT_MS = 5000;
const unsigned long ACT_TIMEOUT_MS    = 5000;

// Heartbeat intervals to boards and server
const unsigned long PING_INTERVAL_MS = 2000;
unsigned long lastPing = 0;

// Helper: send JSON to server via WebSocket
void sendToServer(const JsonDocument& doc) {
  char buffer[512];
  size_t len = serializeJson(doc, buffer);
  ws.sendTXT(buffer, len);
}

// Helper: send raw string to server
void sendToServer(const char* msg) {
  ws.sendTXT(msg);
}

// Send message to a peer via ESP-NOW
void sendToPeer(const uint8_t* peer, const JsonDocument& doc) {
  char buffer[256];
  size_t len = serializeJson(doc, buffer);
  esp_now_send(peer, (uint8_t*)buffer, len);
}

// Callback for ESP-NOW incoming messages
// ESP-NOW receive callback.  The signature changed in ESP-IDF v5.x to include
// esp_now_recv_info_t with metadata about the sender.  We use info->src_addr
// to determine which board sent the message.
void onNowRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (!incomingData || len <= 0) return;
  String msg((const char*)incomingData, len);
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, msg);
  if (err) {
    Serial.print("ESP-NOW JSON error: "); Serial.println(err.c_str());
    return;
  }
  // Determine sender type by comparing src_addr
  bool fromSensor  = false;
  bool fromActuator= false;
  if (info && info->src_addr) {
    fromSensor   = memcmp(info->src_addr, SENSOR_MAC, 6) == 0;
    fromActuator = memcmp(info->src_addr, ACTUATOR_MAC, 6) == 0;
  }
  if (fromSensor) {
    lastSensorHB = millis();
  } else if (fromActuator) {
    lastActHB = millis();
  }
  // Forward all messages to server (include RSSI)
  doc["rssi"] = WiFi.RSSI();
  sendToServer(doc);
}

// Callback for WebSocket events
void onWsEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket connected");
      // Identify to server
      {
        StaticJsonDocument<128> doc;
        doc["type"] = "bridge_connected";
        doc["id"]   = "MASTER";
        sendToServer(doc);
      }
      break;
    case WStype_TEXT: {
      // Receive command from server; forward to corresponding peer
      String msg((char*)payload, length);
      StaticJsonDocument<512> doc;
      if (deserializeJson(doc, msg) != DeserializationError::Ok) {
        Serial.println("WS parse error");
        return;
      }
      // Determine type of message
      const char* mtype = doc["type"];
      if (!mtype) return;
      // Command messages include a "dst" field
      if (strcmp(mtype, "cmd") == 0) {
        const char* dst = doc["dst"];
        if (!dst) return;
        if (strcmp(dst, "ACT") == 0) {
          sendToPeer(ACTUATOR_MAC, doc);
        } else if (strcmp(dst, "SENS") == 0) {
          sendToPeer(SENSOR_MAC, doc);
        }
      } else if (strcmp(mtype, "config_set") == 0) {
        // Configuration messages target a board via "target"
        const char* target = doc["target"];
        if (!target) return;
        if (strcmp(target, "ACT") == 0) {
          sendToPeer(ACTUATOR_MAC, doc);
        } else if (strcmp(target, "SENS") == 0) {
          sendToPeer(SENSOR_MAC, doc);
        }
      }
      break;
    }
    case WStype_PING:
    case WStype_PONG:
    case WStype_BIN:
    case WStype_ERROR:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.printf("Connected. IP address: %s Channel: %d\n", WiFi.localIP().toString().c_str(), WiFi.channel());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    ESP.restart();
  }
  esp_now_register_recv_cb(onNowRecv);

  // Add peers
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, SENSOR_MAC, 6);
  peer.channel = WiFi.channel();
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  memcpy(peer.peer_addr, ACTUATOR_MAC, 6);
  esp_now_add_peer(&peer);

  // Initialize WebSocket
  ws.begin(WS_HOST, WS_PORT, WS_PATH);
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(2000);
}

void loop() {
  ws.loop();
  unsigned long now = millis();
  // Periodically send ping to boards (keep alive) and to server via WS
  if (now - lastPing >= PING_INTERVAL_MS) {
    lastPing = now;
    // Ping sensors and actuator
    StaticJsonDocument<64> doc;
    doc["type"] = "ping";
    doc["from"] = "MASTER";
    sendToPeer(SENSOR_MAC, doc);
    sendToPeer(ACTUATOR_MAC, doc);
    // Optionally send heartbeat to server
    StaticJsonDocument<64> hb;
    hb["type"] = "hb";
    hb["from"] = "MASTER";
    sendToServer(hb);
  }
  // Check timeouts and notify server if boards offline
  if ((millis() - lastSensorHB) > SENSOR_TIMEOUT_MS) {
    StaticJsonDocument<128> doc;
    doc["type"] = "status";
    doc["board"] = "SENS";
    doc["state"] = "OFFLINE";
    sendToServer(doc);
    // Reset lastSensorHB to avoid spamming
    lastSensorHB = millis();
  }
  if ((millis() - lastActHB) > ACT_TIMEOUT_MS) {
    StaticJsonDocument<128> doc;
    doc["type"] = "status";
    doc["board"] = "ACT";
    doc["state"] = "OFFLINE";
    sendToServer(doc);
    lastActHB = millis();
  }
  delay(10);
}
