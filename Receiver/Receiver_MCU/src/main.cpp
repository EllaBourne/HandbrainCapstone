#include <WiFi.h>

void setup(){
  Serial.begin(115200);
  delay(1000);
  WiFi.mode(WIFI_STA);
  Serial.println('MacAddress: ');
  Serial.println(WiFi.macAddress());

}

void loop(){

}