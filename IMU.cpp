// File: IMU.cpp

#include "IMU.h"
#include "IMU_Calibration.h"
#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>

static MPU6050 mpu;
static QMC5883LCompass compass;

// Last raw mag (µT-ish) read from sensor (before calibration)
float imu_last_raw_mag[3] = {0.0f, 0.0f, 0.0f};

bool IMU_Init() {
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        //Serial.println("[IMU] MPU6050 not found");
        return false;
    }
    compass.init();
    IMU_Calibration::begin();
    return true;
}

bool IMU_Read(IMUData &data) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    // --- Raw read from MPU6050 ---
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert to physical units
    float rawAcc[3];
    rawAcc[0] = ax / 16384.0f * 9.81f;
    rawAcc[1] = ay / 16384.0f * 9.81f;
    rawAcc[2] = az / 16384.0f * 9.81f;

    data.acc[0] = rawAcc[0];
    data.acc[1] = rawAcc[1];
    data.acc[2] = rawAcc[2];

    data.gyro[0] = gx / 131.0f;
    data.gyro[1] = gy / 131.0f;
    data.gyro[2] = gz / 131.0f;

    // --- Raw read from QMC5883L ---
    compass.read();
    float rawMag[3];
    rawMag[0] = compass.getX();
    rawMag[1] = compass.getY();
    rawMag[2] = compass.getZ();

    data.mag[0] = rawMag[0];
    data.mag[1] = rawMag[1];
    data.mag[2] = rawMag[2];

    // Save raw mag for diagnostics (before calibration)
    imu_last_raw_mag[0] = rawMag[0];
    imu_last_raw_mag[1] = rawMag[1];
    imu_last_raw_mag[2] = rawMag[2];

    // --- Apply calibration ---
    IMU_Calibration::applyCalibration(data);

    // --- Debug output (throttled by DEBUG_HEADING compile-time flag) ---
//#ifdef DEBUG_HEADING
    //Serial.printf("ACC RAW: [%6.2f %6.2f %6.2f] | CAL: [%6.2f %6.2f %6.2f]\n",
                  //rawAcc[0], rawAcc[1], rawAcc[2],
                  //data.acc[0], data.acc[1], data.acc[2]);

    //Serial.printf("MAG RAW: [%7.1f %7.1f %7.1f] | CAL: [%7.1f %7.1f %7.1f]\n",
                  //rawMag[0], rawMag[1], rawMag[2],
                 // data.mag[0], data.mag[1], data.mag[2]);
//#endif

    return true;
}

