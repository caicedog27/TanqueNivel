
#include "sensor_common.h"
const char* BOARD_ID = "SENS_TANQUE"; const char* SENSOR_ID = "tank";
void setup(){ boot_ms = millis(); Serial.begin(115200); US.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); ensureWiFi(); wsConnect(BOARD_ID); hello(BOARD_ID, "DYP-A01 TANQUE"); }
void loop(){ ws.loop(); ensureWiFi(); float mm; bool got = readBinaryFrame(mm); if(!got) got = readAsciiFrame(mm); if(got) pushVal(mm); if(millis()-last_send_ms>=1000){ float filt = trimmedMean(); sendSample(SENSOR_ID, filt); last_send_ms = millis(); } delay(1); }
