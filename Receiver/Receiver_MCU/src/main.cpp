/*#include <WiFi.h>

void setup(){
  Serial.begin(115200);
  delay(1000);
  WiFi.mode(WIFI_STA);
  Serial.println('MacAddress: ');
  Serial.println(WiFi.macAddress());

}

void loop(){

}*/
/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/  
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h>
#include <string>

// structure matches transmitter
typedef struct imuReadings {
  float acc_x, acc_y, acc_z;
  float gyr_x, gyr_y, gyr_z;
} imuReadings;

imuReadings imuData;

// callback function that runs when data is received
//uint8_t * mac= pointer to mac address, uint8_t * incomingData = pointer to incoming raw bytes
void receivedStatus(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&imuData, incomingData, sizeof(imuData)); //copy raw bytes into struct format
  
  // printing 6 values
  // Serial.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n",
  //               imuData.acc_x, imuData.acc_y, imuData.acc_z,
  //               imuData.gyr_x, imuData.gyr_y, imuData.gyr_z);
  // Converting data to JSON string:
  string jsonData = "{\"imuData\": {\"acc_x\": "+imuData.acc_x+", \"acc_y\": "+imuData.acc_y+", \"acc_z\": "+imuData.acc_y+ \
                    ", \"gyr_X\": "+imuData.gyr_x+", \"gyr_y\": "+imuData.gyr_y+", \"gyr_z\": "+imuData.gyr_z+"}}";
  // print JSON to COM 5 port:
  Serial.println(jsonData);
}
 
void setup() {
  // init serial monitor
  Serial.begin(115200);
  
  // setting device as wifi station
  WiFi.mode(WIFI_STA);

  // initializing espnow protoocols
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
//tells espnow to call function when data arrives, usies register receive vallback function
//this is what makes it automatic
  esp_now_register_recv_cb(esp_now_recv_cb_t(receivedStatus));
  
  Serial.println("Receiver ready - waiting for IMU data...");
}
 
void loop() {
  //dont need bc of callback
}