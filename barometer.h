#ifndef BAROMETER_H
#define BAROMETER_H

#include <Wire.h>
#include <MS5611.h>

// === SETTINGS VARIABLES ===
extern bool useWMA;
extern bool useEMA;
extern bool useKalmanAltitude;
extern bool useKalmanVS;
extern bool enableOutlierFilter;
extern float vSmoothed;
extern float altitude;
extern bool useIMUFusion;   // if true, fuses IMU vertical accel for quicker V/S responsiveness

// Adjustable parameters
extern int avgSamples;
extern int outlierThreshold;
extern float qnh;

// === FUNCTION PROTOTYPES ===
void initBarometer();
void updateBarometer();
float kalmanUpdate(float previousEstimate, float measuredValue);

#endif
