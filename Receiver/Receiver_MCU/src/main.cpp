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

#define GET_MAC 0 // flag to output MAC for this device

// structure matches transmitter
typedef struct imuReadings {
  float acc_x, acc_y, acc_z;
  float gyr_x, gyr_y, gyr_z;
} imuReadings;

typedef struct buttonValues {
  bool button_1, button_2, button_3, button_4;
} buttonValues;

typedef struct unityPacket {
    imuReadings imuData;
    buttonValues buttonData;
} unityPacket;

unityPacket unityData;
// imuReadings imuData;
// buttonValues buttonData;

// eMag receiver mac address (TBD)
// TODO: Find MAC address of the receiving ESP
uint8_t broadcastAddress[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// callback function that runs when data is received
//uint8_t * mac= pointer to mac address, uint8_t * incomingData = pointer to incoming raw bytes
void receivedStatus(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&unityData, incomingData, sizeof(unityData)); //copy raw bytes into struct format
  
  // printing 6 values inJSON object format
  Serial.printf(
      "{\"imuData\": {\"acc_x\": %.2f, \"acc_y\": %.2f, \"acc_z\": %.2f, \"gyr_x\": %.2f, \"gyr_y\": %.2f, \"gyr_z\": %.2f},\
\"buttonData\": {\"button_1\": %d,\"button_2\": %d,\"button_3\": %d,\"button_4\": %d}}\r\n",
      unityData.imuData.acc_x, unityData.imuData.acc_y, unityData.imuData.acc_z,
      unityData.imuData.gyr_x, unityData.imuData.gyr_y, unityData.imuData.gyr_z,
      unityData.buttonData.button_1, unityData.buttonData.button_2, unityData.buttonData.button_3, unityData.buttonData.button_4);
}

// callback when data is sent,autoruns after each transmission to say if successful or not
void sentStatus(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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

  // Tell ESP-NOW to call back when sending a message
  esp_now_register_send_cb(esp_now_send_cb_t(sentStatus));
  
  Serial.println("Receiver ready - waiting for IMU data...");
}
 
void loop() {
  //use to read PWM data from Serial and send it to another ESP32

  // //TODO Test and write code that sends to the other ESP
  // String serial_receive;
  // serial_receive = Serial.readString();

  // // Temp, print the received 
  // Serial.println(serial_receive);

  // using espnow send function to send serial data
    //esp_err_t = data type for ESP error
    //uint8_t * esnures sending as raw bytes
    // esp_err_t status = esp_now_send(broadcastAddress, (uint8_t *) &serial_receive, sizeof(serial_receive));
    // if (status == ESP_OK) {
    //     Serial.println("Sent with success");
    // }
    // else {
    //     Serial.println("Error sending the data");
    // }
    // delay(70);


}