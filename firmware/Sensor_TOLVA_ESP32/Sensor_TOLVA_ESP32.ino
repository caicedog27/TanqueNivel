
#include "sensor_common.h"
const char* BOARD_ID = "SENS_TOLVA"; const char* SENSOR_ID = "hopper";
void setup(){
  boot_ms = millis();
  Serial.begin(115200);
  while(!Serial && millis() - boot_ms < 2000){}
  Serial.println("[BOOT] Sensor TOLVA starting");
  beginSensorInterface();
  Serial.println("[US] Serial interface initialized");
  ensureWiFi();
  wsConnect(BOARD_ID, "DYP-A01 TOLVA");
}
void loop(){
  ws.loop();
  ensureWiFi();
  if(sensorWarningPrinted && millis() - lastSensorRead_ms > sensorTimeout_ms * 3){
    recoverSensorLink();
  }
  float mm;
  if(readSensor(mm)){
    pushVal(mm);
  }
  if(sensorErrorCount > 50){
    Serial.printf("[US] Communication errors detected: %u\n", sensorErrorCount);
    sensorErrorCount = 0;
  }
  if(millis() - last_send_ms >= 1000){
    if(wCount > 0){
      uint32_t age = (lastSensorRead_ms == 0) ? 0 : (millis() - lastSensorRead_ms);
      if(sensorDataFresh(sensorTimeout_ms * 2)){
        float filt = filteredValue();
        if(wsReadyForSamples){
          sendSample(SENSOR_ID, filt);
          Serial.printf("[DATA] Sent filtered level: %.1f mm\n", filt);
        } else {
          static uint32_t lastHandshakeLog = 0;
          if(millis() - lastHandshakeLog > 5000){
            Serial.println("[WS] Waiting for handshake before sending samples");
            lastHandshakeLog = millis();
          }
        }
      } else {
        static uint32_t lastStaleLog = 0;
        if(millis() - lastStaleLog > 3000){
          Serial.printf("[DATA] Skipping stale sensor data (age %lu ms)\n", (unsigned long)age);
          lastStaleLog = millis();
        }
      }
    } else {
      Serial.println("[DATA] Waiting for valid sensor samples before reporting");
    }
    last_send_ms = millis();
  }
  delay(2);
}
