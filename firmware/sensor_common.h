
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include <cstring>
#include <stdlib.h>

const char* FW_VERSION = "UX15";
uint32_t boot_ms = 0;


const char* WIFI_SSID = "PLANTA_VIVO_NAVE";
const char* WIFI_PASS = "901878434-1";
const char* WS_HOST  = "192.168.1.68";
const uint16_t WS_PORT = 8000;
const char* BOARD_TOKEN = "test_ws_board_2025_ABCDEF";

const int SENSOR_RX_PIN = 16;
const int SENSOR_TRIGGER_PIN = 17;
#ifndef SENSOR_ID
extern const char* SENSOR_ID;
#endif
#if defined(CONFIG_IDF_TARGET_ESP32C3)
HardwareSerial& US = Serial1;
#else
HardwareSerial& US = Serial2;
#endif

// According to the DYP-A01 timing diagram the sensor can provide a new sample
// every ~100 ms. We trigger slightly slower to stay within the guaranteed
// response window while keeping the reported level reactive.
const uint32_t SENSOR_TRIGGER_INTERVAL_MS = 120;
const uint32_t SENSOR_TRIGGER_PULSE_US = 2000;
const uint32_t SENSOR_POLL_MIN_INTERVAL_MS = 5;
const uint32_t SENSOR_WARNING_AFTER_MS = 5000;
const uint32_t SENSOR_FAULT_AFTER_MS = 10000;
const uint32_t SENSOR_STATUS_THROTTLE_MS = 2500;

WebSocketsClient ws;
uint32_t last_send_ms = 0;
const int WN = 7; float windowVals[WN]; int wCount=0; float last_mm = 0;

const char* wsBoardId = nullptr;
const char* wsBoardName = nullptr;
bool wsReadyForSamples = false;
bool wsHelloAcked = false;
bool wsHelloInFlight = false;
uint32_t wsHelloSent_ms = 0;
uint32_t wsHelloLog_ms = 0;

enum SensorHealthState : uint8_t {
  SENSOR_HEALTH_OK = 0,
  SENSOR_HEALTH_WARNING = 1,
  SENSOR_HEALTH_FAULT = 2,
  SENSOR_HEALTH_UNKNOWN = 3
};

SensorHealthState lastHealthState = SENSOR_HEALTH_UNKNOWN;
char lastHealthReason[32] = "";
uint32_t lastHealthReport_ms = 0;

// --- Advanced filtering configuration ---
const float MIN_LEVEL_MM = 50.0f;
const float MAX_LEVEL_MM = 4500.0f;
const float MAX_STEP_MM = 200.0f;
const float MIN_MEASUREMENT_VAR = 25.0f;        // (5 mm)^2
const float MAX_MEASUREMENT_VAR = 250000.0f;    // (500 mm)^2
const float PROCESS_NOISE_PER_SECOND = 4000.0f; // mm^2 / s
const float OUTLIER_THRESHOLD_MM = 150.0f;
const float OUTLIER_REJECTION_GAIN = 6.0f;

uint32_t lastFilterUpdate_ms = 0;
bool filterInitialized = false;
float filtered_mm = 0;
float kalmanError = 0;

void flushSensorBuffer();
void resetWindow();
void resetFilterState();
void beginSensorInterface();
void triggerSensorPulse();
void wsSendJson(DynamicJsonDocument &doc);
void monitorWsHandshake();

void publishSensorHealth(SensorHealthState state, const char* reason){
  if(!wsReadyForSamples){
    lastHealthState = state;
    if(reason){
      strncpy(lastHealthReason, reason, sizeof(lastHealthReason)-1);
      lastHealthReason[sizeof(lastHealthReason)-1] = '\0';
    } else {
      lastHealthReason[0] = '\0';
    }
    return;
  }
  uint32_t now = millis();
  if(lastHealthState == state){
    bool sameReason = false;
    if((reason == nullptr || reason[0] == '\0') && lastHealthReason[0] == '\0'){
      sameReason = true;
    } else if(reason != nullptr){
      sameReason = (strncmp(reason, lastHealthReason, sizeof(lastHealthReason)) == 0);
    }
    if(sameReason && (now - lastHealthReport_ms) < SENSOR_STATUS_THROTTLE_MS){
      return;
    }
  }
  DynamicJsonDocument d(256);
  d["type"] = "sensor:status";
  if(SENSOR_ID){ d["sensor_id"] = SENSOR_ID; }
  const char* statusStr = "UNKNOWN";
  switch(state){
    case SENSOR_HEALTH_OK: statusStr = "OK"; break;
    case SENSOR_HEALTH_WARNING: statusStr = "WARNING"; break;
    case SENSOR_HEALTH_FAULT: statusStr = "FAULT"; break;
    default: statusStr = "UNKNOWN"; break;
  }
  d["status"] = statusStr;
  if(reason && reason[0]){
    d["reason"] = reason;
  }
  d["rssi"] = WiFi.RSSI();
  d["uptime_s"] = (millis() - boot_ms) / 1000;
  wsSendJson(d);
  lastHealthReport_ms = now;
  lastHealthState = state;
  if(reason){
    strncpy(lastHealthReason, reason, sizeof(lastHealthReason)-1);
    lastHealthReason[sizeof(lastHealthReason)-1] = '\0';
  } else {
    lastHealthReason[0] = '\0';
  }
}

void sensorHealthOk(){
  if(lastHealthState != SENSOR_HEALTH_OK || lastHealthReason[0] != '\0'){
    publishSensorHealth(SENSOR_HEALTH_OK, "");
  }
}

void ensureWiFi(){
  static uint8_t attempt = 0;
  static uint32_t lastAttempt = 0;
  if(WiFi.status() == WL_CONNECTED){
    return;
  }
  if(lastAttempt != 0 && millis() - lastAttempt < 2000){
    return;
  }
  attempt++;
  lastAttempt = millis();
  Serial.printf("[WiFi] Connecting to %s (attempt %u)\n", WIFI_SSID, attempt);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  wl_status_t status = WiFi.status();
  while(status != WL_CONNECTED && millis() - t0 < 15000){
    delay(250);
    status = WiFi.status();
  }
  if(status == WL_CONNECTED){
    IPAddress ip = WiFi.localIP();
    Serial.printf("[WiFi] Connected. IP: %s, RSSI: %d dBm, channel: %d\n", ip.toString().c_str(), WiFi.RSSI(), WiFi.channel());
    Serial.printf("[WiFi] Gateway: %s, DNS: %s\n", WiFi.gatewayIP().toString().c_str(), WiFi.dnsIP().toString().c_str());
    attempt = 0;
  } else {
    Serial.printf("[WiFi] Failed to connect within timeout (status=%d, RSSI=%d)\n", (int)status, WiFi.RSSI());
    Serial.println("[WiFi] Retrying after backoff...");
    WiFi.disconnect(true);
  }
}
void wsSendJson(DynamicJsonDocument &doc){ String out; serializeJson(doc, out); ws.sendTXT(out); }
void hello(const char* id, const char* name){
  DynamicJsonDocument d(256);
  d["type"] = "hello";
  d["id"] = id;
  d["kind"] = "SENSOR";
  d["name"] = name;
  d["token"] = BOARD_TOKEN;
  d["fw"] = FW_VERSION;
  d["mac"] = WiFi.macAddress();
  d["rssi"] = WiFi.RSSI();
  d["uptime_s"] = (millis() - boot_ms) / 1000;
  wsSendJson(d);
  wsHelloAcked = false;
  wsReadyForSamples = false;
  wsHelloInFlight = true;
  wsHelloSent_ms = millis();
  wsHelloLog_ms = wsHelloSent_ms;
  Serial.printf("[WS] Hello sent (id=%s, name=%s, fw=%s, RSSI=%d)\n", id, name ? name : "?", FW_VERSION, WiFi.RSSI());
}
void sendSample(const char* sensor, float mm){ if(!wsReadyForSamples){ return; } DynamicJsonDocument d(256); d["type"]="sensor"; d["sensor_id"]=sensor; d["mm"]=mm; d["rssi"]=WiFi.RSSI(); d["uptime_s"]=(millis()-boot_ms)/1000; wsSendJson(d); }

void handleWsText(uint8_t * payload, size_t length){
  if(payload == nullptr || length == 0){
    Serial.println("[WS] Empty text frame received");
    return;
  }
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if(err){
    Serial.printf("[WS] Message (raw): %.*s\n", (int)length, (const char*)payload);
    Serial.printf("[WS] JSON parse error: %s\n", err.c_str());
    return;
  }
  const char* type = doc["type"] | "";
  if(strcmp(type, "hello:ack") == 0){
    wsHelloAcked = true;
    wsReadyForSamples = true;
    wsHelloInFlight = false;
    const char* status = doc["status"] | "ok";
    const char* message = doc["message"] | "";
    const char* server = doc["server"] | "server";
    Serial.printf("[WS] Hello acknowledged (status=%s) by %s\n", status, server);
    if(message && message[0]){
      Serial.printf("[WS] Server message: %s\n", message);
    }
  } else if(strcmp(type, "error") == 0){
    const char* reason = doc["reason"] | "unknown";
    int code = doc["code"].is<int>() ? doc["code"].as<int>() : -1;
    if(code >= 0){
      Serial.printf("[WS] Server error (code %d): %s\n", code, reason);
    } else {
      Serial.printf("[WS] Server error: %s\n", reason);
    }
  } else {
    Serial.printf("[WS] Message: %.*s\n", (int)length, (const char*)payload);
  }
}

void logWsDisconnection(uint8_t * payload, size_t length){
  uint16_t code = 0;
  String reason;
  if(payload != nullptr && length >= 2){
    code = (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
    if(length > 2){
      reason.reserve(length - 2);
      for(size_t i = 2; i < length; ++i){
        char c = (char)payload[i];
        if(c >= 32 && c <= 126){
          reason += c;
        }
      }
    }
  }
  if(code > 0){
    if(reason.length() > 0){
      Serial.printf("[WS] Disconnected from server (code %u, reason=%s)\n", code, reason.c_str());
    } else {
      Serial.printf("[WS] Disconnected from server (code %u)\n", code);
    }
  } else {
    Serial.println("[WS] Disconnected from server");
  }
}

void onWsEvent(WStype_t type, uint8_t * payload, size_t length){
  switch(type){
    case WStype_CONNECTED:
      Serial.printf("[WS] Connected to %s:%u\n", WS_HOST, WS_PORT);
      wsReadyForSamples = false;
      wsHelloAcked = false;
      wsHelloInFlight = false;
      if(wsBoardId != nullptr && wsBoardName != nullptr){
        hello(wsBoardId, wsBoardName);
      } else {
        Serial.println("[WS] Board identity not set, cannot send hello");
      }
      break;
    case WStype_DISCONNECTED:
      logWsDisconnection(payload, length);
      if(wsHelloInFlight && !wsHelloAcked){
        Serial.println("[WS] Handshake failed before ACK was received");
      }
      wsReadyForSamples = false;
      wsHelloAcked = false;
      wsHelloInFlight = false;
      break;
    case WStype_ERROR:
      Serial.println("[WS] Error event received");
      if(payload && length > 0){
        Serial.printf("[WS] Error payload: %.*s\n", (int)length, (const char*)payload);
      }
      wsReadyForSamples = false;
      wsHelloAcked = false;
      wsHelloInFlight = false;
      break;
    case WStype_TEXT:
      handleWsText(payload, length);
      break;
    case WStype_PONG:
    case WStype_PING:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

void wsConnect(const char* id, const char* name){ wsBoardId = id; wsBoardName = name; wsReadyForSamples = false; wsHelloAcked = false; wsHelloInFlight = false; String path=String("/ws/board/")+id+"?token="+BOARD_TOKEN; ws.begin(WS_HOST, WS_PORT, path.c_str()); ws.onEvent(onWsEvent); ws.setReconnectInterval(2000);
  ws.enableHeartbeat(15000, 3000, 2); }

uint32_t lastSensorRead_ms = 0;
uint32_t lastSensorTrigger_ms = 0;
uint32_t sensorTimeout_ms = 2000;
uint16_t sensorErrorCount = 0;
bool sensorWarningPrinted = false;

void beginSensorInterface(){
  pinMode(SENSOR_TRIGGER_PIN, OUTPUT);
  digitalWrite(SENSOR_TRIGGER_PIN, HIGH);
  US.begin(9600, SERIAL_8N1, SENSOR_RX_PIN, -1);
  flushSensorBuffer();
  lastSensorTrigger_ms = millis() - SENSOR_TRIGGER_INTERVAL_MS;
}

void triggerSensorPulse(){
  digitalWrite(SENSOR_TRIGGER_PIN, LOW);
  delayMicroseconds(SENSOR_TRIGGER_PULSE_US);
  digitalWrite(SENSOR_TRIGGER_PIN, HIGH);
  lastSensorTrigger_ms = millis();
}

bool readBinaryFrame(float &mm){
  enum ParseState : uint8_t { WAIT_HEADER = 0, READ_HIGH, READ_LOW, READ_CHECKSUM };
  static ParseState state = WAIT_HEADER;
  static uint8_t highByte = 0;
  static uint8_t lowByte = 0;
  static uint8_t desyncCounter = 0;
  while(US.available()){
    uint8_t b = US.read();
    switch(state){
      case WAIT_HEADER:
        if(b == 0xFF){
          state = READ_HIGH;
          desyncCounter = 0;
        } else {
          desyncCounter++;
          if(desyncCounter > 20){
            flushSensorBuffer();
            desyncCounter = 0;
          }
        }
        break;
      case READ_HIGH:
        highByte = b;
        state = READ_LOW;
        break;
      case READ_LOW:
        lowByte = b;
        state = READ_CHECKSUM;
        break;
      case READ_CHECKSUM:{
        uint8_t checksum = (uint8_t)((0xFF + highByte + lowByte) & 0xFF);
        state = WAIT_HEADER;
        if(checksum == b){
          mm = (float)((highByte << 8) | lowByte);
          return true;
        }
        sensorErrorCount++;
        if(sensorErrorCount % 8 == 0){
          flushSensorBuffer();
        }
        break;
      }
    }
  }
  return false;
}
bool readAsciiFrame(float &mm){
  static char buffer[16];
  static uint8_t index = 0;
  while(US.available()){
    char c = US.read();
    if(c == '\r'){
      continue;
    }
    if(c == '\n'){
      if(index > 0){
        buffer[index] = '\0';
        mm = (float)atoi(buffer);
        index = 0;
        return true;
      }
      index = 0;
      continue;
    }
    if(isDigit(c)){
      if(index < sizeof(buffer) - 1){
        buffer[index++] = c;
      }
    } else {
      index = 0;
      sensorErrorCount++;
    }
  }
  return false;
}

bool readSensor(float &mm){
  static uint32_t lastPoll_ms = 0;
  uint32_t now = millis();

  if(now - lastSensorTrigger_ms >= SENSOR_TRIGGER_INTERVAL_MS){
    triggerSensorPulse();
  }

  if(now - lastPoll_ms < SENSOR_POLL_MIN_INTERVAL_MS && US.available() == 0){
    return false;
  }
  lastPoll_ms = now;

  bool anyFrame = false;
  bool validFrame = false;
  float latestValid = 0.0f;

  while(true){
    float candidate = 0.0f;
    bool got = readBinaryFrame(candidate);
    if(!got){
      got = readAsciiFrame(candidate);
    }
    if(!got){
      break;
    }
    anyFrame = true;
    if(candidate <= 0){
      sensorErrorCount++;
      if(sensorErrorCount % 10 == 0){
        Serial.println("[US] Invalid reading received (<= 0 mm)");
      }
      continue;
    }
    if(candidate > 6000){
      sensorErrorCount++;
      if(sensorErrorCount % 10 == 0){
        Serial.println("[US] Invalid reading received (> 6000 mm)");
      }
      continue;
    }
    latestValid = candidate;
    validFrame = true;
  }

  if(validFrame){
    mm = latestValid;
    lastSensorRead_ms = millis();
    sensorWarningPrinted = false;
    sensorHealthOk();
    sensorErrorCount = 0;
    return true;
  }

  if(anyFrame){
    return false;
  }

  if(!sensorWarningPrinted && millis() - lastSensorRead_ms > sensorTimeout_ms){
    Serial.println("[US] No readings received from ultrasonic sensor");
    sensorWarningPrinted = true;
    publishSensorHealth(SENSOR_HEALTH_WARNING, "timeout");
  }
  uint32_t since = millis() - lastSensorRead_ms;
  if(since > SENSOR_WARNING_AFTER_MS && lastHealthState == SENSOR_HEALTH_OK){
    publishSensorHealth(SENSOR_HEALTH_WARNING, "timeout");
  }
  if(since > SENSOR_FAULT_AFTER_MS){
    publishSensorHealth(SENSOR_HEALTH_FAULT, "timeout");
  }
  if(sensorWarningPrinted && since > sensorTimeout_ms * 2){
    resetWindow();
    resetFilterState();
  }
  return false;
}

bool sensorDataFresh(uint32_t maxAge_ms){
  if(lastSensorRead_ms == 0){
    return false;
  }
  return (millis() - lastSensorRead_ms) <= maxAge_ms;
}

struct WindowStats{
  float mean;
  float variance;
  int count;
};

WindowStats computeWindowStats(){
  WindowStats stats{last_mm, 0.0f, 0};
  if(wCount == 0) return stats;
  int n = (wCount < WN) ? wCount : WN;
  float buf[WN];
  for(int i=0;i<n;i++) buf[i] = windowVals[i];
  for(int i=1;i<n;i++){
    float key = buf[i];
    int j = i - 1;
    while(j >= 0 && buf[j] > key){
      buf[j + 1] = buf[j];
      j--;
    }
    buf[j + 1] = key;
  }
  int trim = (n >= 5) ? 1 : 0;
  int start = trim;
  int end = n - trim;
  if(start >= end){
    start = 0;
    end = n;
  }
  float sum = 0.0f;
  int count = 0;
  for(int i=start;i<end;i++){
    sum += buf[i];
    count++;
  }
  float mean = (count > 0) ? (sum / count) : last_mm;
  float variance = 0.0f;
  if(count > 1){
    for(int i=start;i<end;i++){
      float diff = buf[i] - mean;
      variance += diff * diff;
    }
    variance /= (count - 1);
  }
  stats.mean = mean;
  stats.variance = variance;
  stats.count = count;
  return stats;
}

void resetFilterState(){
  filterInitialized = false;
  filtered_mm = 0.0f;
  kalmanError = 0.0f;
  lastFilterUpdate_ms = millis();
}

void resetWindow(){
  wCount = 0;
  last_mm = 0.0f;
}

void pushVal(float mm){
  if(last_mm > 0){
    float delta = mm - last_mm;
    if(delta > MAX_STEP_MM) mm = last_mm + MAX_STEP_MM;
    else if(delta < -MAX_STEP_MM) mm = last_mm - MAX_STEP_MM;
  }
  if(mm < MIN_LEVEL_MM) mm = MIN_LEVEL_MM;
  if(mm > MAX_LEVEL_MM) mm = MAX_LEVEL_MM;
  last_mm = mm;
  if(wCount < WN){
    windowVals[wCount++] = mm;
  } else {
    for(int i=1;i<WN;i++) windowVals[i-1] = windowVals[i];
    windowVals[WN-1] = mm;
  }
}

float filteredValue(){
  WindowStats stats = computeWindowStats();
  uint32_t now = millis();
  float dt = (lastFilterUpdate_ms == 0) ? 0.0f : (now - lastFilterUpdate_ms) * 0.001f;
  if(dt <= 0.0f) dt = 0.001f;
  if(!filterInitialized){
    filtered_mm = stats.mean;
    kalmanError = (stats.variance > MIN_MEASUREMENT_VAR) ? stats.variance : MIN_MEASUREMENT_VAR;
    filterInitialized = true;
  } else {
    kalmanError += PROCESS_NOISE_PER_SECOND * dt;
    float measurementVar = stats.variance;
    if(measurementVar < MIN_MEASUREMENT_VAR) measurementVar = MIN_MEASUREMENT_VAR;
    if(measurementVar > MAX_MEASUREMENT_VAR) measurementVar = MAX_MEASUREMENT_VAR;
    if(fabsf(stats.mean - filtered_mm) > OUTLIER_THRESHOLD_MM){
      measurementVar *= OUTLIER_REJECTION_GAIN;
    }
    float gain = kalmanError / (kalmanError + measurementVar);
    filtered_mm = filtered_mm + gain * (stats.mean - filtered_mm);
    kalmanError = (1.0f - gain) * kalmanError;
  }
  lastFilterUpdate_ms = now;
  return filtered_mm;
}

void flushSensorBuffer(){
  while(US.available()){
    US.read();
  }
}

void recoverSensorLink(){
  Serial.println("[US] Attempting to recover ultrasonic sensor link");
  US.end();
  delay(20);
  beginSensorInterface();
  resetWindow();
  resetFilterState();
  lastSensorRead_ms = 0;
  sensorWarningPrinted = false;
  sensorErrorCount = 0;
}

void monitorWsHandshake(){
  if(!wsHelloInFlight){
    return;
  }
  uint32_t now = millis();
  uint32_t elapsed = now - wsHelloSent_ms;
  if(now - wsHelloLog_ms > 3000){
    Serial.printf("[WS] Waiting for hello ACK... (%lu ms elapsed, WiFi RSSI=%d, status=%d)\n",
                  (unsigned long)elapsed, WiFi.RSSI(), (int)WiFi.status());
    wsHelloLog_ms = now;
  }
  if(elapsed > 20000){
    Serial.println("[WS] Hello ACK not received after 20s; will rely on reconnect logic");
    wsHelloInFlight = false;
    wsHelloAcked = false;
    wsReadyForSamples = false;
  }
}
