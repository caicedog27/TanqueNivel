
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"
#include <esp_ota_ops.h>
#include <esp_partition.h>

const char* FW_VERSION = "UX14";
uint32_t boot_ms = 0;


const char* WIFI_SSID = "PLANTA_VIVO_NAVE";
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
bool wsHelloAcked = false;
bool wsHelloInFlight = false;
uint32_t wsHelloSent_ms = 0;
uint32_t wsHelloLog_ms = 0;

void setup_wdt(){
  esp_task_wdt_config_t cfg = { .timeout_ms = 10000, .idle_core_mask = (1<<portNUM_PROCESSORS)-1, .trigger_panic = true };
  esp_err_t err = esp_task_wdt_init(&cfg);
  if(err == ESP_ERR_INVALID_STATE){ /* ya estaba */ }
  esp_task_wdt_add(NULL);
}

void ensureWiFi(){
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA); WiFi.setSleep(false); WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  wl_status_t status = WiFi.status();
  while (status != WL_CONNECTED && millis() - t0 < 15000){
    esp_task_wdt_reset();
    delay(120);
    status = WiFi.status();
  }
  if(status == WL_CONNECTED){
    IPAddress ip = WiFi.localIP();
    Serial.printf("[WiFi] Connected. IP: %s, RSSI: %d dBm, channel: %d\n", ip.toString().c_str(), WiFi.RSSI(), WiFi.channel());
    Serial.printf("[WiFi] Gateway: %s, DNS: %s\n", WiFi.gatewayIP().toString().c_str(), WiFi.dnsIP().toString().c_str());
  } else {
    Serial.printf("[WiFi] Connection attempt timed out (status=%d, RSSI=%d)\n", (int)status, WiFi.RSSI());
    WiFi.disconnect(true);
  }
}

void wsSendJson(DynamicJsonDocument &d){ String out; serializeJson(d, out); ws.sendTXT(out); }

void hello(){
  DynamicJsonDocument d(256);
  d["type"]="hello"; d["id"]=BOARD_ID; d["kind"]="ACTUATOR"; d["name"]="Actuador Bomba"; d["token"]=BOARD_TOKEN; d["fw"]=FW_VERSION; d["mac"]=WiFi.macAddress(); d["rssi"]=WiFi.RSSI(); d["uptime_s"]=(millis()-boot_ms)/1000;
  wsSendJson(d);
  wsHelloAcked = false;
  wsHelloInFlight = true;
  wsHelloSent_ms = millis();
  wsHelloLog_ms = wsHelloSent_ms;
  Serial.printf("[WS] Hello sent (board=%s, fw=%s, RSSI=%d)\n", BOARD_ID, FW_VERSION, WiFi.RSSI());
}

void setPump(bool on){
  pump_on = on;
  digitalWrite(MAIN_PIN, on ? HIGH : LOW);
  digitalWrite(AIR_PIN,  on ? HIGH : LOW);
  if(!on){ digitalWrite(PULSE_PIN, LOW); pulse_state=false; }
  else{ last_toggle_ms = millis(); pulse_state=true; digitalWrite(PULSE_PIN, HIGH); pulses_total++; }
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
      Serial.printf("[WS] Disconnected (code %u, reason=%s)\n", code, reason.c_str());
    } else {
      Serial.printf("[WS] Disconnected (code %u)\n", code);
    }
  } else {
    Serial.println("[WS] Disconnected from server");
  }
}

void onWsEvent(WStype_t type, uint8_t * payload, size_t length){
  switch(type){
    case WStype_CONNECTED:
      Serial.printf("[WS] Connected to %s:%u\n", WS_HOST, WS_PORT);
      wsHelloAcked = false;
      wsHelloInFlight = false;
      hello();
      break;
    case WStype_DISCONNECTED:
      logWsDisconnection(payload, length);
      if(wsHelloInFlight && !wsHelloAcked){
        Serial.println("[WS] Handshake did not complete before disconnect");
      }
      wsHelloAcked = false;
      wsHelloInFlight = false;
      break;
    case WStype_ERROR:
      Serial.println("[WS] Error event received");
      wsHelloAcked = false;
      wsHelloInFlight = false;
      break;
    case WStype_TEXT:{
      DynamicJsonDocument d(512);
      if(deserializeJson(d, payload, length)){
        Serial.printf("[WS] JSON parse error on actuator message: %.*s\n", (int)length, (const char*)payload);
        return;
      }
      const char* t = d["type"] | "";
      if(strcmp(t, "hello:ack") == 0){
        wsHelloAcked = true;
        wsHelloInFlight = false;
        const char* status = d["status"] | "ok";
        const char* message = d["message"] | "";
        Serial.printf("[WS] Hello acknowledged (status=%s)\n", status);
        if(message && message[0]){
          Serial.printf("[WS] Server message: %s\n", message);
        }
      } else if(strcmp(t,"actuator:pump")==0){
        const char* v = d["value"] | "OFF"; setPump(strcmp(v,"ON")==0);
        { DynamicJsonDocument a(192); a["type"]="ack"; a["cmd"]="actuator:pump"; a["value"]=v; a["ts"]=millis(); wsSendJson(a); }
      } else if(strcmp(t,"actuator:set")==0){
        on_ms = (uint32_t)(d["on_ms"] | on_ms);
        off_ms = (uint32_t)(d["off_ms"] | off_ms);
        { DynamicJsonDocument a(192); a["type"]="ack"; a["cmd"]="actuator:set"; a["on_ms"]=on_ms; a["off_ms"]=off_ms; a["ts"]=millis(); wsSendJson(a); }
      } else if(strcmp(t,"error")==0){
        const char* reason = d["reason"] | "unknown";
        int code = d["code"].is<int>() ? d["code"].as<int>() : -1;
        if(code >= 0){
          Serial.printf("[WS] Server error (code %d): %s\n", code, reason);
        } else {
          Serial.printf("[WS] Server error: %s\n", reason);
        }
      } else {
        Serial.printf("[WS] Message: %.*s\n", (int)length, (const char*)payload);
      }
      break;
    }
    default:
      break;
  }
}

void wsConnect(){
  String path = String("/ws/board/")+BOARD_ID+"?token="+BOARD_TOKEN;
  ws.begin(WS_HOST, WS_PORT, path.c_str());
  ws.onEvent(onWsEvent);
  ws.setReconnectInterval(2000);
  ws.enableHeartbeat(15000, 3000, 2);
  wsHelloAcked = false;
  wsHelloInFlight = false;
}

void sendStats(){
  DynamicJsonDocument d(256);
  d["type"]="actuator:stats";
  d["pulses_total"]=pulses_total;
  d["runtime_ms_total"]=runtime_ms_total;
  d["rssi"]=WiFi.RSSI();
  d["uptime_s"]=(millis()-boot_ms)/1000;
  wsSendJson(d);
}

void setup(){
  boot_ms = millis();
  pinMode(PULSE_PIN, OUTPUT); pinMode(AIR_PIN, OUTPUT); pinMode(MAIN_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW); digitalWrite(AIR_PIN, LOW); digitalWrite(MAIN_PIN, LOW);
  Serial.begin(115200);
  uint32_t serial_wait_start = millis();
  while(!Serial && (millis() - serial_wait_start) < 1500){ delay(10); }
  const esp_partition_t* running = esp_ota_get_running_partition();
  if(running){
    Serial.printf("[Boot] Flash size: %u bytes\n", ESP.getFlashChipSize());
    Serial.printf("[Boot] Running partition '%s' at 0x%06x with max size %u bytes\n",
                  running->label, running->address, running->size);
  } else {
    Serial.println("[Boot] Unable to read running partition information");
  }
  setup_wdt(); ensureWiFi(); wsConnect();
  last_runtime_tick = millis();
}

void loop(){
  ws.loop(); ensureWiFi();
  if(wsHelloInFlight){
    uint32_t now = millis();
    if(now - wsHelloLog_ms > 3000){
      Serial.printf("[WS] Waiting for hello ACK... (%lu ms elapsed, WiFi RSSI=%d, status=%d)\n",
                    (unsigned long)(now - wsHelloSent_ms), WiFi.RSSI(), (int)WiFi.status());
      wsHelloLog_ms = now;
    }
    if(now - wsHelloSent_ms > 20000){
      Serial.println("[WS] Hello ACK not received after 20s; keeping socket for reconnect");
      wsHelloInFlight = false;
      wsHelloAcked = false;
    }
  }
  esp_task_wdt_reset();
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
  delay(1);
}
