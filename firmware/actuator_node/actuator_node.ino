#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// Output pin for Solid State Relay (SSR) controlling the pump
#define PIN_SSR 25
// Output pin for pulse relay. When pump is ON, this pin toggles at the configured rate (pulses per minute).
#define PIN_PULSE 26

// WiFi credentials and server endpoint
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";
const char* WS_HOST  = "192.168.1.100"; // change to server IP
const uint16_t WS_PORT = 8000;
const char* WS_PATH = "/ws/board/ACT"; // WebSocket endpoint for actuator

// Preferences for storing persistent settings
Preferences prefs;

// Failsafe timer variables
volatile unsigned long lastCommandMs = 0;
volatile unsigned long currentTTLms = 3000;
volatile bool pumpOn = false;

// Pulse generation variables
float pulseRate = 1.0f;          // pulses per minute (default)
bool pulseOnState = false;       // current output state of the pulse relay
unsigned long lastPulseToggle = 0; // last time pulse pin toggled

// Heartbeat interval for sending status back to server
const unsigned long HB_INTERVAL_MS = 2000;
unsigned long lastHb = 0;

WebSocketsClient ws;

void onWsEvent(WStype_t type, uint8_t *payload, size_t length);
void sendState(const char* reason);
void sendToServer(const JsonDocument& doc);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);
  pinMode(PIN_PULSE, OUTPUT);
  digitalWrite(PIN_PULSE, LOW);
  // Load previous state if exists
  prefs.begin("act", true);
  pumpOn = prefs.getBool("pumpOn", false);
  currentTTLms = prefs.getULong("ttl", 3000);
  pulseRate = prefs.getFloat("pulseRate", 1.0f);
  prefs.end();
  digitalWrite(PIN_SSR, pumpOn ? HIGH : LOW);
  if (pumpOn) lastCommandMs = millis();

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
  unsigned long now = millis();
  ws.loop();
  // Failsafe: turn off pump if TTL expired
  if (pumpOn && (now - lastCommandMs > currentTTLms)) {
    pumpOn = false;
    digitalWrite(PIN_SSR, LOW);
    // also ensure pulse output is off
    if (pulseOnState) {
      pulseOnState = false;
      digitalWrite(PIN_PULSE, LOW);
    }
    Serial.println("TTL expired -> Pump OFF");
    sendState("TTL_EXPIRED");
  }
  // Pulse generation: toggle pulse relay while pump is on
  if (pumpOn && pulseRate > 0.0f) {
    // compute half period in milliseconds
    float periodMs = 60000.0f / pulseRate; // full period (ms) per pulse
    unsigned long halfPeriod = (unsigned long)(periodMs / 2.0f);
    if (now - lastPulseToggle >= halfPeriod) {
      pulseOnState = !pulseOnState;
      digitalWrite(PIN_PULSE, pulseOnState ? HIGH : LOW);
      lastPulseToggle = now;
    }
  } else {
    // ensure pulse output is off when pump is off or rate is zero
    if (pulseOnState) {
      pulseOnState = false;
      digitalWrite(PIN_PULSE, LOW);
    }
  }
  // Send heartbeat periodically
  if (now - lastHb >= HB_INTERVAL_MS) {
    lastHb = now;
    sendState("HB");
  }
}

// WebSocket callback for commands/config from server
void onWsEvent(WStype_t type, uint8_t *payload, size_t length) {
  if (type != WStype_TEXT) return;
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length) != DeserializationError::Ok) {
    Serial.println("JSON parse error on actuator");
    return;
  }
  const char* mtype = doc["type"];
  if (!mtype) return;
  if (strcmp(mtype, "cmd") == 0) {
    const char* dst = doc["dst"];
    if (dst && String(dst) != "ACT") return;
    const char* cmd = doc["cmd"];
    const char* value = doc["value"];
    uint32_t ttl = doc["ttl_ms"] | 0;
    if (cmd && strcmp(cmd, "PUMP") == 0 && value) {
      currentTTLms = ttl ? ttl : 3000;
      if (strcmp(value, "ON") == 0) {
        pumpOn = true;
        digitalWrite(PIN_SSR, HIGH);
        lastCommandMs = millis();
        Serial.println("Pump ON via command");
        sendState("CMD_ON");
      } else {
        pumpOn = false;
        digitalWrite(PIN_SSR, LOW);
        Serial.println("Pump OFF via command");
        sendState("CMD_OFF");
      }
      prefs.begin("act", false);
      prefs.putBool("pumpOn", pumpOn);
      prefs.putULong("ttl", currentTTLms);
      prefs.end();
    }
  } else if (strcmp(mtype, "config_set") == 0) {
    const char* target = doc["target"];
    if (target && String(target) == "ACT") {
      JsonObject payload = doc["payload"];
      if (payload.containsKey("pulse_rate")) {
        float newRate = payload["pulse_rate"].as<float>();
        if (newRate < 0.001f) newRate = 0.0f;
        pulseRate = newRate;
        prefs.begin("act", false);
        prefs.putFloat("pulseRate", pulseRate);
        prefs.end();
        Serial.printf("Pulse rate updated: %.3f pulses/min\n", pulseRate);
        sendState("PULSE_RATE_SET");
      }
    }
  }
}

// Send pump state back to server
void sendState(const char* reason) {
  StaticJsonDocument<128> doc;
  doc["type"] = "ack";
  doc["from"] = "ACT";
  doc["state"] = pumpOn ? "ON" : "OFF";
  doc["reason"] = reason;
  sendToServer(doc);
}

void sendToServer(const JsonDocument& doc) {
  char buffer[256];
  size_t len = serializeJson(doc, buffer);
  ws.sendTXT(buffer, len);
}
