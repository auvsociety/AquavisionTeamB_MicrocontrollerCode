#ifndef SENSOR_INTERFACE_HPP
#define SENSOR_INTERFACE_HPP

// #include <mpu6050.hpp>
#include <Adafruit_BNO08x.h>

// #include "Adafruit_HMC5883_U.h"
#include "MS5837.h"

#include "main.hpp"
// #include "sensor_math.hpp"

// #define ACCEL_ID 0x8700A
// #define MAG_ID 0x8700B

// #define FXOS8700_ADDRESS 0x1F
// #define MS5837_ADDR 0x76
// #define MPU6050_ADDRESS 0x68

extern Adafruit_BNO08x  bno08x;

extern MS5837 depth_sensor;

extern struct euler_t ypr;

void initializeIMU();
void initializeDepthSensor();
// void callUpdateOffset(bool& calib_status);
void updateIMUReadings(float& ax, float& ay, float& az, float& gx, float& gy,
                       float& gz, float& mx, float& my, float& mz);
void updateDepthSensorReadings(float& depth);


void updateOrientation(float &roll,float &pitch, float &yaw);
void getAccuracy(int &accuracy);

// extern MPU6050 gyro;

// extern Adafruit_HMC5883_Unified mag ;


#endif  // SENSOR_INTERFACE_HPP