
#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

const char* FW_VERSION = "UX15";
uint32_t boot_ms = 0;


const char* WIFI_SSID = "PLANTA_VIVO_NAVE";
const char* WIFI_PASS = "901878434-1";
const char* WS_HOST  = "192.168.1.68";
const uint16_t WS_PORT = 8000;
const char* BOARD_TOKEN = "test_ws_board_2025_ABCDEF";

const int SENSOR_RX_PIN = 16;
const int SENSOR_TRIGGER_PIN = 17;
#if defined(CONFIG_IDF_TARGET_ESP32C3)
HardwareSerial& US = Serial1;
#else
HardwareSerial& US = Serial2;
#endif

const uint32_t SENSOR_TRIGGER_INTERVAL_MS = 250;
const uint32_t SENSOR_TRIGGER_PULSE_US = 2000;

WebSocketsClient ws;
uint32_t last_send_ms = 0;
const int WN = 7; float windowVals[WN]; int wCount=0; float last_mm = 0;

const char* wsBoardId = nullptr;
const char* wsBoardName = nullptr;
bool wsReadyForSamples = false;

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
void sendSample(const char* sensor, float mm){ if(!wsReadyForSamples){ return; } DynamicJsonDocument d(256); d["type"]="sensor"; d["sensor_id"]=sensor; d["mm"]=mm; d["rssi"]=WiFi.RSSI(); d["uptime_s"]=(millis()-boot_ms)/1000; wsSendJson(d); }
void onWsEvent(WStype_t type, uint8_t * payload, size_t length){
  switch(type){
    case WStype_CONNECTED:
      Serial.println("[WS] Connected to server");
      wsReadyForSamples = false;
      if(wsBoardId != nullptr && wsBoardName != nullptr){
        hello(wsBoardId, wsBoardName);
        wsReadyForSamples = true;
      } else {
        Serial.println("[WS] Board identity not set, cannot send hello");
      }
      break;
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected from server");
      wsReadyForSamples = false;
      break;
    case WStype_ERROR:
      Serial.println("[WS] Error event received");
      wsReadyForSamples = false;
      break;
    case WStype_TEXT:
      Serial.printf("[WS] Message: %.*s\n", (int)length, (const char*)payload);
      break;
    default:
      break;
  }
}
void wsConnect(const char* id, const char* name){ wsBoardId = id; wsBoardName = name; wsReadyForSamples = false; String path=String("/ws/board/")+id+"?token="+BOARD_TOKEN; ws.begin(WS_HOST, WS_PORT, path.c_str()); ws.onEvent(onWsEvent); ws.setReconnectInterval(2000);
  ws.enableHeartbeat(15000, 3000, 2); }

uint32_t lastSensorRead_ms = 0;
uint32_t lastSensorTrigger_ms = 0;
uint32_t sensorTimeout_ms = 2000;
uint16_t sensorErrorCount = 0;
bool sensorWarningPrinted = false;

void flushSensorBuffer();
void beginSensorInterface();
void triggerSensorPulse();

void flushSensorBuffer(){
  while(US.available()){
    US.read();
  }
}

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
  while(US.available() >= 4){
    int b = US.read();
    if(b != 0xFF) continue;
    int H = US.read(), L = US.read(), S = US.read();
    if( ((uint8_t)((0xFF + H + L) & 0xFF)) == (uint8_t)S ){
      mm = (float)((H<<8) | L);
      return true;
    } else {
      sensorErrorCount++;
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
  uint32_t now = millis();
  if((now - lastSensorTrigger_ms >= SENSOR_TRIGGER_INTERVAL_MS) && (US.available() < 4)){
    triggerSensorPulse();
  }
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
  return false;
}

float trimmedMean(){
  if(wCount==0) return last_mm;
  float buf[WN]; int n = wCount<WN?wCount:WN;
  for(int i=0;i<n;i++) buf[i]=windowVals[i];
  for(int i=0;i<n;i++) for(int j=i+1;j<n;j++) if(buf[j]<buf[i]){ float t=buf[i]; buf[i]=buf[j]; buf[j]=t; }
  int cut = (n>=5)?1:0; int a=cut, b=n-cut; if(a>=b){ a=0; b=n; }
  float sum=0; int cnt=0; for(int i=a;i<b;i++){ sum+=buf[i]; cnt++; } return (cnt>0)?(sum/cnt):last_mm;
}
void pushVal(float mm){
  if(last_mm>0){ float delta = mm - last_mm; if(delta > 120) mm = last_mm + 120; else if(delta < -120) mm = last_mm - 120; }
  if(mm < 50) mm = 50; if(mm > 4500) mm = 4500; last_mm = mm;
  if(wCount < WN){ windowVals[wCount++] = mm; }
  else{ for(int i=1;i<WN;i++) windowVals[i-1] = windowVals[i]; windowVals[WN-1] = mm; }
}
bool filterInitialized = false;
float filtered_mm = 0;
float filteredValue(){
  float tm = trimmedMean();
  if(!filterInitialized){
    filtered_mm = tm;
    filterInitialized = true;
  } else {
    const float alpha = 0.4f;
    filtered_mm = alpha * tm + (1.0f - alpha) * filtered_mm;
  }
  return filtered_mm;
}
void recoverSensorLink(){
  Serial.println("[US] Attempting to recover ultrasonic sensor link");
  US.end();
  delay(20);
  beginSensorInterface();
  lastSensorRead_ms = millis();
  sensorWarningPrinted = false;
  sensorErrorCount = 0;
}
