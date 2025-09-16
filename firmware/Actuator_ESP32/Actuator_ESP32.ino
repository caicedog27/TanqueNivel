
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"

const char* WIFI_SSID = "PLANTA_VIVI_NAVE";
const char* WIFI_PASS = "901878434-1";
const char* WS_HOST  = "192.168.1.68";
const uint16_t WS_PORT = 8000;
const char* BOARD_ID = "ACT-01";
const char* BOARD_TOKEN = "test_ws_board_2025_ABCDEF";

const int PULSE_PIN = 25;
const int AIR_PIN   = 26;
const int MAIN_PIN  = 27;

WebSocketsClient ws;
bool pump_on = false;
uint32_t on_ms = 100, off_ms = 233;
uint32_t last_toggle_ms = 0; bool pulse_state = false;
uint32_t pulses_total = 0, runtime_ms_total = 0, last_runtime_tick = 0, last_stats_send = 0;
uint32_t last_wifi_report = 0, wifiReconnects = 0, lastWifiAttempt = 0;
bool wifiInit = false;

int wifiQuality(int32_t rssi){ if(rssi<=-100) return 0; if(rssi>=-50) return 100; return 2*(rssi+100); }

void setup_wdt(){
  esp_task_wdt_config_t cfg = { .timeout_ms = 10000, .idle_core_mask = (1<<portNUM_PROCESSORS)-1, .trigger_panic = true };
  esp_err_t err = esp_task_wdt_init(&cfg);
  if(err == ESP_ERR_INVALID_STATE){ /* ya estaba */ }
  esp_task_wdt_add(NULL);
}

void ensureWiFi(){
  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) return;
  uint32_t now = millis();
  if(!wifiInit){ WiFi.mode(WIFI_STA); WiFi.persistent(false); WiFi.setSleep(false); WiFi.setAutoReconnect(true); wifiInit = true; }
  if(now - lastWifiAttempt < 500) return;
  lastWifiAttempt = now;
  if(status == WL_CONNECT_FAILED || status == WL_CONNECTION_LOST || status == WL_NO_SSID_AVAIL || status == WL_DISCONNECTED){ WiFi.disconnect(false, true); delay(50); }
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiReconnects++;
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000){
    esp_task_wdt_reset();
    delay(150);
  }
}

void wsSendJson(DynamicJsonDocument &d){ String out; serializeJson(d, out); ws.sendTXT(out); }

void sendWifiStatus(){
  if(!ws.isConnected()) return;
  DynamicJsonDocument d(256);
  d["type"]="wifi:status";
  wl_status_t st = WiFi.status();
  int32_t rssi = (st==WL_CONNECTED)?WiFi.RSSI():-120;
  d["rssi"]=rssi;
  d["quality"]=wifiQuality(rssi);
  d["reconnects"]=wifiReconnects;
  d["uptime_ms"]=millis();
  d["connected"]=(st==WL_CONNECTED);
  d["ip"]=WiFi.localIP().toString();
  wsSendJson(d);
}

void hello(){
  DynamicJsonDocument d(256);
  d["type"]="hello"; d["id"]=BOARD_ID; d["kind"]="ACTUATOR"; d["name"]="Actuador Bomba"; d["token"]=BOARD_TOKEN;
  wsSendJson(d);
}

void setPump(bool on){
  pump_on = on;
  digitalWrite(MAIN_PIN, on ? HIGH : LOW);
  digitalWrite(AIR_PIN,  on ? HIGH : LOW);
  if(!on){ digitalWrite(PULSE_PIN, LOW); pulse_state=false; }
  else{ last_toggle_ms = millis(); pulse_state=true; digitalWrite(PULSE_PIN, HIGH); pulses_total++; }
}

void onWsEvent(WStype_t type, uint8_t * payload, size_t length){
  if(type == WStype_CONNECTED){ hello(); last_wifi_report = 0; sendWifiStatus(); }
  else if(type == WStype_DISCONNECTED){ last_wifi_report = 0; }
  else if(type == WStype_TEXT){
    DynamicJsonDocument d(512); if(deserializeJson(d, payload, length)) return;
    const char* t = d["type"] | "";
    if(strcmp(t,"actuator:pump")==0){
      const char* v = d["value"] | "OFF"; setPump(strcmp(v,"ON")==0);
    }else if(strcmp(t,"actuator:set")==0){
      on_ms = (uint32_t)(d["on_ms"] | on_ms);
      off_ms = (uint32_t)(d["off_ms"] | off_ms);
    }
  }
}

void wsConnect(){
  String path = String("/ws/board/")+BOARD_ID+"?token="+BOARD_TOKEN;
  ws.begin(WS_HOST, WS_PORT, path.c_str());
  ws.onEvent(onWsEvent);
  ws.enableHeartbeat(15000, 3000, 2);
  ws.setReconnectInterval(1500);
}

void sendStats(){
  DynamicJsonDocument d(256);
  d["type"]="actuator:stats";
  d["pulses_total"]=pulses_total;
  d["runtime_ms_total"]=runtime_ms_total;
  wsSendJson(d);
}

void setup(){
  pinMode(PULSE_PIN, OUTPUT); pinMode(AIR_PIN, OUTPUT); pinMode(MAIN_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW); digitalWrite(AIR_PIN, LOW); digitalWrite(MAIN_PIN, LOW);
  Serial.begin(115200);
  setup_wdt(); ensureWiFi(); wsConnect();
  last_runtime_tick = millis();
}

void loop(){
  ws.loop(); ensureWiFi(); esp_task_wdt_reset();
  const uint32_t now = millis();
  if(pump_on){
    runtime_ms_total += (now - last_runtime_tick);
    last_runtime_tick = now;
    const uint32_t elapsed = now - last_toggle_ms;
    if(pulse_state){
      if(elapsed >= on_ms){ pulse_state=false; digitalWrite(PULSE_PIN, LOW); last_toggle_ms = now; }
    } else {
      if(elapsed >= off_ms){ pulse_state=true; digitalWrite(PULSE_PIN, HIGH); last_toggle_ms = now; pulses_total++; }
    }
  } else { last_runtime_tick = now; }
  if(now - last_stats_send > 1200){ sendStats(); last_stats_send = now; }
  if(now - last_wifi_report > 10000){ sendWifiStatus(); last_wifi_report = now; }
  delay(1);
}
