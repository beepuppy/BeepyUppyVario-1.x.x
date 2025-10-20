// File: IMU.h

#pragma once
#include <Arduino.h>

struct IMUData {
    float acc[3];   // m/s²
    float gyro[3];  // deg/s
    float mag[3];   // µT (calibrated)
};

// Latest raw magnetometer reading (before calibration). Useful for diagnostics.
// Defined in IMU.cpp
extern float imu_last_raw_mag[3];

bool IMU_Init();                 // Call once in setup()
bool IMU_Read(IMUData &data);    // Call in loop()

