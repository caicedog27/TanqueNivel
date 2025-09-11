#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// Output pin for Solid State Relay (SSR) controlling the pump
#define PIN_SSR 25
// Output pin for pulse relay. When pump is ON, this pin toggles at the configured rate (pulses per minute).
#define PIN_PULSE 26

// MAC address of master/gateway board
uint8_t MASTER_MAC[] = {0xA0, 0xB7, 0x65, 0x28, 0x71, 0x3C};

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

// Heartbeat interval for sending status back to master
const unsigned long HB_INTERVAL_MS = 2000;
unsigned long lastHb = 0;

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len);
void sendState(const char* reason);

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
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
  }
  // Register receive callback with new signature (esp_now_recv_info_t*)
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, MASTER_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add master peer");
  }
}

void loop() {
  unsigned long now = millis();
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

// Callback for receiving commands from master.  New ESP-IDF v5.x signature
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (!incomingData || len <= 0) return;
  String msg((const char*)incomingData, len);
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, msg) != DeserializationError::Ok) {
    Serial.println("JSON parse error on actuator");
    return;
  }
  const char* type = doc["type"];
  if (!type) return;
  // Command from server forwarded by master
  if (strcmp(type, "cmd") == 0) {
    const char* dst = doc["dst"];
    if (dst && String(dst) != "ACT") return; // Not for us
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
      // Persist last state and TTL
      prefs.begin("act", false);
      prefs.putBool("pumpOn", pumpOn);
      prefs.putULong("ttl", currentTTLms);
      prefs.end();
    }
  } else if (strcmp(type, "ping") == 0) {
    sendState("HB");
  } else if (strcmp(type, "config_set") == 0) {
    // Handle configuration for actuator (pulse rate)
    const char* target = doc["target"];
    if (target && String(target) == "ACT") {
      JsonObject payload = doc["payload"];
      if (payload.containsKey("pulse_rate")) {
        float newRate = payload["pulse_rate"].as<float>();
        if (newRate < 0.001f) newRate = 0.0f;
        pulseRate = newRate;
        // Persist pulse rate
        prefs.begin("act", false);
        prefs.putFloat("pulseRate", pulseRate);
        prefs.end();
        Serial.printf("Pulse rate updated: %.3f pulses/min\n", pulseRate);
        sendState("PULSE_RATE_SET");
      }
    }
  }
}

// Send pump state back to master
void sendState(const char* reason) {
  StaticJsonDocument<128> doc;
  doc["type"] = "ack";
  doc["from"] = "ACT";
  doc["state"] = pumpOn ? "ON" : "OFF";
  doc["reason"] = reason;
  char buffer[128];
  size_t len = serializeJson(doc, buffer);
  esp_now_send(MASTER_MAC, (uint8_t*)buffer, len);
}
