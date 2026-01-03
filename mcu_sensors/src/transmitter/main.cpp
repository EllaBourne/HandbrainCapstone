#include <Arduino.h>
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
}
