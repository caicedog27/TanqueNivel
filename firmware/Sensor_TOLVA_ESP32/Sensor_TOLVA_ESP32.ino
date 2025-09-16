
#include "sensor_common.h"
const char* BOARD_ID = "SENS_TOLVA"; const char* SENSOR_ID = "hopper";
void setup(){ Serial.begin(115200); US.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); ensureWiFi(); wsConnect(BOARD_ID); hello(BOARD_ID, "DYP-A01 TOLVA"); }
void loop(){
  ws.loop(); ensureWiFi();
  float mm; bool got = readBinaryFrame(mm); if(!got) got = readAsciiFrame(mm); if(got) pushVal(mm);
  if(millis()-last_send_ms>=1000){ float filt = trimmedMean(); sendSample(SENSOR_ID, filt); last_send_ms = millis(); }
  if(millis() - lastWifiReportMs >= 10000){ sendWifiStatus(BOARD_ID); lastWifiReportMs = millis(); }
  delay(1);
}
