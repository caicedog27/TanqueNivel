
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

const char* WIFI_SSID = "PLANTA_VIVI_NAVE";
const char* WIFI_PASS = "901878434-1";
const char* WS_HOST  = "192.168.1.68";
const uint16_t WS_PORT = 8000;
const char* BOARD_TOKEN = "test_ws_board_2025_ABCDEF";

const int RX_PIN = 16;
const int TX_PIN = 17;
HardwareSerial& US = Serial2;

WebSocketsClient ws;
uint32_t last_send_ms = 0;
const int WN = 7; float windowVals[WN]; int wCount=0; float last_mm = 0;
uint32_t lastWifiReportMs = 0; uint32_t wifiReconnects = 0; uint32_t lastWifiAttemptMs = 0; bool wifiInit = false;
String gHelloId; String gHelloName;

int wifiQuality(int32_t rssi){ if(rssi<=-100) return 0; if(rssi>=-50) return 100; return 2*(rssi+100); }

void ensureWiFi(){
  wl_status_t status = WiFi.status();
  if(status == WL_CONNECTED) return;
  uint32_t now = millis();
  if(!wifiInit){ WiFi.mode(WIFI_STA); WiFi.persistent(false); WiFi.setSleep(false); WiFi.setAutoReconnect(true); wifiInit = true; }
  if(now - lastWifiAttemptMs < 500) return;
  lastWifiAttemptMs = now;
  if(status == WL_CONNECT_FAILED || status == WL_CONNECTION_LOST || status == WL_NO_SSID_AVAIL || status == WL_DISCONNECTED){ WiFi.disconnect(false, true); delay(50); }
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiReconnects++;
  uint32_t t0 = millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-t0<15000){ delay(150); yield(); }
}

void wsSendJson(DynamicJsonDocument &doc){ String out; serializeJson(doc, out); ws.sendTXT(out); }
void sendWifiStatus(const char* boardId){ if(!ws.isConnected()) return; DynamicJsonDocument d(256); d["type"]="wifi:status"; wl_status_t st = WiFi.status(); int32_t rssi = (st==WL_CONNECTED)?WiFi.RSSI():-120; d["rssi"]=rssi; d["quality"]=wifiQuality(rssi); d["reconnects"]=wifiReconnects; d["uptime_ms"]=millis(); d["connected"]= (st==WL_CONNECTED); d["ip"]=WiFi.localIP().toString(); if(boardId && boardId[0]) d["id"]=boardId; wsSendJson(d); }
void hello(const char* id, const char* name){ gHelloId = id; gHelloName = name; DynamicJsonDocument d(256); d["type"]="hello"; d["id"]=id; d["kind"]="SENSOR"; d["name"]=name; d["token"]=BOARD_TOKEN; wsSendJson(d); }
void sendSample(const char* sensor, float mm){ DynamicJsonDocument d(256); d["type"]="sensor"; d["sensor_id"]=sensor; d["mm"]=mm; wsSendJson(d); }
void onWsEvent(WStype_t type, uint8_t * payload, size_t length){
  if(type==WStype_CONNECTED){ if(gHelloId.length()){ hello(gHelloId.c_str(), gHelloName.c_str()); lastWifiReportMs = 0; sendWifiStatus(gHelloId.c_str()); } }
  else if(type==WStype_DISCONNECTED){ lastWifiReportMs = 0; }
}
void wsConnect(const char* id){ String path=String("/ws/board/")+id+"?token="+BOARD_TOKEN; ws.begin(WS_HOST, WS_PORT, path.c_str()); ws.onEvent(onWsEvent); ws.enableHeartbeat(15000, 3000, 2); ws.setReconnectInterval(1500); }

bool readBinaryFrame(float &mm){
  while(US.available() >= 4){
    int b = US.read();
    if(b != 0xFF) continue;
    int H = US.read(), L = US.read(), S = US.read();
    if( ((uint8_t)((0xFF + H + L) & 0xFF)) == (uint8_t)S ){ mm = (float)((H<<8) | L); return true; }
  }
  return false;
}
bool readAsciiFrame(float &mm){
  static String line;
  while(US.available()){
    char c = US.read();
    if(c=='\n'){ line.trim(); if(line.length()>0){ mm = line.toInt(); line=""; return true; } line=""; }
    else if(isDigit(c)){ line += c; } else if(c=='\r'){ } else { line=""; }
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
