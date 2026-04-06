/*#include <Arduino.h>
#include <Wire.h>
#include "I2C_MPU6886.h"

//pins for ESP32 PICO
#define SDA_PIN 26
#define SCL_PIN 32

I2C_MPU6886 imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire);

void setup() {
    Serial.begin(115200); //bode rate
    delay(1000);

    Wire.begin(SDA_PIN, SCL_PIN);   // pins for specific pico board (SDA = 26, SCL = 32)

    imu.begin();  // initialize MPU6886

}

void loop() {
    float acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z; //for accelerometer and gyroscope

    imu.getAccel(&acc_x, &acc_y, &acc_z); //built in IMU functions
    imu.getGyro(&gyr_x, &gyr_y, &gyr_z);

    Serial.printf("%.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z);

    delay(100); // 10 samples/sec
}*/
/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h> //espnow needs the wifi functions but doesnt rely on wifi
#include <Arduino.h>
#include <Wire.h>
#include "I2C_MPU6886.h"
#include "driver/gpio.h"


//pins defiend for the pico
// #define SDA_PIN 26 //I2C serial
// #define SCL_PIN 32 //I2C clock

//pins defiend for the ESP32-C3
#define SDA_PIN 0 //I2C serial
#define SCL_PIN 1 //I2C clock
#define FREQUENCY 200000 // I2C Speed

#define GET_MAC 0 // flag to output MAC for this device

//I2C_MPU6886 imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire); //imu object


// receiver mac address
// TODO: replace this when we get the new controllers
uint8_t broadcastAddress[] = {0xD4, 0xD4, 0xDA, 0x83, 0x95, 0x08};

typedef struct imuReadings {
    float acc_x,acc_y, acc_z;
    float gyr_x, gyr_y, gyr_z;
} imuReadings;

typedef struct buttonValues {
    bool button_1, button_2, button_3, button_4;
} buttonValues;

typedef struct unityPacket {
    imuReadings imuData;
    buttonValues buttonData;
} unityPacket;

unityPacket unityData; //new variable of type unityPacket, contains imuReadings and buttonValues
// imuReadings imuData; //new variable of type imuReadings
// buttonValues buttonData;

esp_now_peer_info_t peerInfo; //espnow connection info object (?)

// callback when data is sent,autoruns after each transmission to say if successful or not
void sentStatus(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("Last Packet Send Status: Delivery Fail");
    } else {
        // Serial.print("\r\nLast Packet Send Status:\t");
        // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    }
}

void setupBMI() {
    // Set up the BMI330 IMU device.

    // Check that chip ID matches
    Wire.beginTransmission(0x00);
    // byte LSB = Wire.read();
    // byte MSB = Wire.read();
    // if (LSB == 0x48) {
    //     Serial.println("LSB is what we want!");
    // } else if (MSB == 0x48) {
    //     Serial.println("MSB is what we want!");
    // } else {
    //     Serial.println("Uh Oh.");
    // }
}

imuReadings readAccelGyro() {
    // Read from the X, Y and Z components of the Gyroscope
    imuReadings accel_gyro = { 0 }; 

    return accel_gyro;
}
 
void setup() {
    
    Serial.begin(115200);
    Serial.printf("Serial Begin!\r\n");
    if(Wire.begin(SDA_PIN, SCL_PIN, FREQUENCY)) {
        Serial.println("Successful Wire Connection!");
    }
    //imu.begin();

    // Initialize BMI330
    //setupBMI();
 
    WiFi.mode(WIFI_STA); //set device as wifi station 

    // Configure Push Buttons as GPIO inputs
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_6, GPIO_MODE_INPUT);
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_INPUT);

    //initialize espnow
    if (esp_now_init() != ESP_OK) { //start protocol, if not successfull, print error
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // after successfull init, register for Send CB  (callback get called auto when something happens) to get status of trasnmitted packet
    //after ESP send data, immediately call sentStatus
    esp_now_register_send_cb(esp_now_send_cb_t(sentStatus));
  
    //setting up connection to receiver
    //peerInfo.peer_addr = peer info MAC address field
    memcpy(peerInfo.peer_addr, broadcastAddress, 6); //6 = 6 bytes
    peerInfo.channel = 0;  //wifi channel 0 allows auto selection
    peerInfo.encrypt = false; //dont need encryption 
  
    //creating peer connection, if fails print error, if succeeds, done setup
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }
}
 
void loop() {
    //reading imu data from sensor 
    //imu.getAccel(&unityData.imuData.acc_x, &unityData.imuData.acc_y, &unityData.imuData.acc_z);
    //imu.getGyro(&unityData.imuData.gyr_x, &unityData.imuData.gyr_y, &unityData.imuData.gyr_z);
    Wire.requestFrom(0x00,2);

    unityData.imuData.acc_x = 0.0f;
    unityData.imuData.acc_y = 0.0f;
    unityData.imuData.acc_z = 0.0f;
    unityData.imuData.gyr_x = 0.0f;
    unityData.imuData.gyr_y = 0.0f;
    unityData.imuData.gyr_z = 0.0f;

    // update unityData.buttonData values with proper code
    unityData.buttonData.button_1 = gpio_get_level(GPIO_NUM_4);
    unityData.buttonData.button_2 = gpio_get_level(GPIO_NUM_5);
    unityData.buttonData.button_3 = gpio_get_level(GPIO_NUM_6);
    unityData.buttonData.button_4 = gpio_get_level(GPIO_NUM_7);

    /*//for debugging
    Serial.printf("Acc: %.2f, %.2f, %.2f , Gyro: %.2f, %.2f, %.2f\n",
            imuData.acc_x, imuData.acc_y, imuData.acc_z,
            imuData.gyr_x, imuData.gyr_y, imuData.gyr_z);

    Serial.printf("B1: %d, B2: %d, B3: %d, B4: %d\r\n",
            gpio_get_level(GPIO_NUM_4), gpio_get_level(GPIO_NUM_5),
            gpio_get_level(GPIO_NUM_6), gpio_get_level(GPIO_NUM_7));
    */
  
    // using espnow send function
    //esp_err_t = data type for ESP error
    //uint8_t * esnures sending as raw bytes
    esp_err_t status = esp_now_send(broadcastAddress, (uint8_t *) &unityData, sizeof(unityData));
    if (status == ESP_OK) {
        //Serial.println("Sent with success");
    }
    else {
        Serial.println("Error sending the data!");
    }
    delay(70);
}
