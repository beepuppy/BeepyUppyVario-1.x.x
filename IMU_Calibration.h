#pragma once
#include <Arduino.h>
#include "IMU.h"

namespace IMU_Calibration {
    // Init sensors and load EEPROM config
    void begin();

    // Drive active calibration state machine (call every loop)
    void update();

    // True if any calibration (mag or fused) is running
    bool isCalibrating();

    // Apply current calibration to raw IMUData
    void applyCalibration(IMUData &data);

    // Magnetometer-only calibration
    void startMagCalibration();

    // Two‑phase fused calibration: timed gyro bias (still) + timed accel/mag (rotation)
    void startFusedCalibration(uint32_t rotationMs = 50000);

    // Manual gyro bias capture (still)
    void startGyroBiasCalibration();

    // Reset all calibration data to defaults
    void resetCalibration();

    // Show the calibration menu panel
    void showCalibMenu();
}
