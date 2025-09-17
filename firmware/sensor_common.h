
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <math.h>

const char* FW_VERSION = "UX15";
uint32_t boot_ms = 0;


const char* WIFI_SSID = "PLANTA_VIVI_NAVE";
const char* WIFI_PASS = "901878434-1";
const char* WS_HOST  = "192.168.1.68";
const uint16_t WS_PORT = 8000;
const char* BOARD_TOKEN = "test_ws_board_2025_ABCDEF";

const int RX_PIN = 16;
const int TX_PIN = 17;
#if defined(CONFIG_IDF_TARGET_ESP32C3)
HardwareSerial& US = Serial1;
#else
HardwareSerial& US = Serial2;
#endif

WebSocketsClient ws;
uint32_t last_send_ms = 0;
const int WN = 7; float windowVals[WN]; int wCount=0; float last_mm = 0;

// --- Advanced filtering configuration ---
const float MIN_LEVEL_MM = 50.0f;
const float MAX_LEVEL_MM = 4500.0f;
const float MAX_STEP_MM = 120.0f;
const float MIN_MEASUREMENT_VAR = 25.0f;        // (5 mm)^2
const float MAX_MEASUREMENT_VAR = 250000.0f;    // (500 mm)^2
const float PROCESS_NOISE_PER_SECOND = 2500.0f; // mm^2 / s
const float OUTLIER_THRESHOLD_MM = 150.0f;
const float OUTLIER_REJECTION_GAIN = 6.0f;

uint32_t lastFilterUpdate_ms = 0;
bool filterInitialized = false;
float filtered_mm = 0;
float kalmanError = 0;

void flushSensorBuffer();
void resetWindow();
void resetFilterState();

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
    Serial.printf("[WiFi] Connected. IP: %s, RSSI: %d\n", ip.toString().c_str(), WiFi.RSSI());
    attempt = 0;
  } else {
    Serial.println("[WiFi] Failed to connect within timeout, retrying...");
    WiFi.disconnect(true);
  }
}
void wsSendJson(DynamicJsonDocument &doc){ String out; serializeJson(doc, out); ws.sendTXT(out); }
void hello(const char* id, const char* name){ DynamicJsonDocument d(256); d["type"]="hello"; d["id"]=id; d["kind"]="SENSOR"; d["name"]=name; d["token"]=BOARD_TOKEN; d["fw"]=FW_VERSION; d["mac"]=WiFi.macAddress(); d["rssi"]=WiFi.RSSI(); d["uptime_s"]=(millis()-boot_ms)/1000; wsSendJson(d); }
void sendSample(const char* sensor, float mm){ DynamicJsonDocument d(256); d["type"]="sensor"; d["sensor_id"]=sensor; d["mm"]=mm; d["rssi"]=WiFi.RSSI(); d["uptime_s"]=(millis()-boot_ms)/1000; wsSendJson(d); }
void onWsEvent(WStype_t type, uint8_t * payload, size_t length){
  switch(type){
    case WStype_CONNECTED:
      Serial.println("[WS] Connected to server");
      break;
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected from server");
      break;
    case WStype_ERROR:
      Serial.println("[WS] Error event received");
      break;
    case WStype_TEXT:
      Serial.printf("[WS] Message: %.*s\n", (int)length, (const char*)payload);
      break;
    default:
      break;
  }
}
void wsConnect(const char* id){ String path=String("/ws/board/")+id+"?token="+BOARD_TOKEN; ws.begin(WS_HOST, WS_PORT, path.c_str()); ws.onEvent(onWsEvent); ws.setReconnectInterval(2000);
  ws.enableHeartbeat(15000, 3000, 2); }

uint32_t lastSensorRead_ms = 0;
uint32_t sensorTimeout_ms = 2000;
uint16_t sensorErrorCount = 0;
bool sensorWarningPrinted = false;

bool readBinaryFrame(float &mm){
  static uint8_t desyncCounter = 0;
  while(US.available() >= 4){
    int b = US.read();
    if(b != 0xFF){
      desyncCounter++;
      if(desyncCounter > 20){
        flushSensorBuffer();
        desyncCounter = 0;
      }
      continue;
    }
    desyncCounter = 0;
    int H = US.read(), L = US.read(), S = US.read();
    if( ((uint8_t)((0xFF + H + L) & 0xFF)) == (uint8_t)S ){
      mm = (float)((H<<8) | L);
      return true;
    } else {
      sensorErrorCount++;
      if(sensorErrorCount % 8 == 0){
        flushSensorBuffer();
      }
    }
  }
  return false;
}
bool readAsciiFrame(float &mm){
  static String line;
  while(US.available()){
    char c = US.read();
    if(c=='\n'){
      line.trim();
      if(line.length()>0){
        mm = line.toInt();
        line="";
        return true;
      }
      line="";
    }
    else if(isDigit(c)){
      line += c;
    } else if(c=='\r'){
    } else {
      line="";
      sensorErrorCount++;
    }
  }
  return false;
}

bool readSensor(float &mm){
  static uint32_t lastPoll_ms = 0;
  uint32_t now = millis();
  if(now - lastPoll_ms < 25 && US.available() == 0){
    return false;
  }
  lastPoll_ms = now;
  bool got = readBinaryFrame(mm);
  if(!got) got = readAsciiFrame(mm);
  if(got){
    lastSensorRead_ms = millis();
    sensorWarningPrinted = false;
    if(mm <= 0){
      sensorErrorCount++;
      if(sensorErrorCount % 10 == 0){
        Serial.println("[US] Invalid reading received (<= 0 mm)");
      }
      return false;
    }
    if(mm > 6000){
      sensorErrorCount++;
      if(sensorErrorCount % 10 == 0){
        Serial.println("[US] Invalid reading received (> 6000 mm)");
      }
      return false;
    }
    sensorErrorCount = 0;
    return true;
  }
  if(lastSensorRead_ms == 0){
    lastSensorRead_ms = millis();
  }
  if(!sensorWarningPrinted && millis() - lastSensorRead_ms > sensorTimeout_ms){
    Serial.println("[US] No readings received from ultrasonic sensor");
    sensorWarningPrinted = true;
  }
  if(sensorWarningPrinted && millis() - lastSensorRead_ms > sensorTimeout_ms * 2){
    resetWindow();
    resetFilterState();
  }
  return false;
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
  US.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  flushSensorBuffer();
  resetWindow();
  resetFilterState();
  lastSensorRead_ms = millis();
  sensorWarningPrinted = false;
}
