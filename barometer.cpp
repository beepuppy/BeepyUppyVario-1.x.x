// barometer.cpp  (replace your existing file with this)
#include "Barometer.h"
#include "gps.h"   // NEW: for gps.getHDOP(), getVDOP(), getSatellites()
#include "Attitude.h"
#include "IMU.h"   // for extern IMUData (we expect a global 'imu' in the main file)

extern IMUData imu;   // 👈 add this line

#include <math.h>

#define MAX_SAMPLES 50
float pressureBuffer[MAX_SAMPLES] = {0};
int pressureIndex = 0;

float vBuffer[MAX_SAMPLES] = {0};
int vIndex = 0;

unsigned long lastUpdate = 0;
float h_prev = 0.0;
float vSmoothed = 0;
float altitude = 0.0;
int avgSamples = 10;  // keep default behavior
float qnh = 1023.0;
bool useKalmanAltitude = false;
bool enableOutlierFilter = true;
int outlierThreshold = 3;
bool useKalmanVS = false;
bool useWMA = false;
bool useEMA = true;

// NEW: optional IMU fusion flag (declare extern in barometer.h)
bool useIMUFusion = false; // default off; enable if you want faster response

MS5611 ms5611;

// === KALMAN FILTER VARIABLES ===
float kalman_altitude = 0;
float kalman_vs = 0;
float kalman_gain = 0.2;

float kalmanUpdate(float previousEstimate, float measuredValue) {
    return previousEstimate + kalman_gain * (measuredValue - previousEstimate);
}

// --- QNH gate constants ---
static const uint8_t  QNH_MIN_SATS_USED   = 6;
static const float    QNH_MAX_HDOP        = 1.5f; // actual
static const float    QNH_MAX_VDOP        = 2.0f; // actual
static bool           qnhLocked           = false;

// --- IMU fusion internal state ---
static float v_accumulator = 0.0f;        // integrated velocity from accel (m/s)
static unsigned long last_accel_ts = 0;   // micros for integration
static const float G_const = 9.81f;       // gravity (m/s^2)
static const float FUSE_ALPHA = 0.45f;    // 0..1 -> weight to barometer; lower -> more accel influence (tuneable)
static const float MAX_ACCEPTED_TILT_DEG = 25.0f; // only use IMU accel-based integration if tilt <= this

// ✅ **Initialize Barometer ONLY ONCE in setup()**
void initBarometer() {
    Wire.begin();
    if (!ms5611.begin()) {
        Serial.println("MS5611 not found");
        while (1);
    }
    ms5611.setOversampling(OSR_ULTRA_HIGH);

    // Seed pressure buffer so averaging starts stable
    ms5611.read();
    float p = ms5611.getPressure();
    for (int i=0;i<avgSamples;i++) pressureBuffer[i] = p;
    for (int i=0;i<avgSamples;i++) vBuffer[i] = 0.0f;
    vSmoothed = 0.0f;
    h_prev = (1.0f - pow(p / qnh, 0.1903f)) * 44330.0f;
    lastUpdate = micros();
    last_accel_ts = micros();
    v_accumulator = 0.0f;
}

// Utility: rotate body accel into world (NED) Z (down) using roll/pitch (deg)
// - input imu.acc[] is body-frame acc in m/s^2 (includes gravity), with Z = down in body frame
static float compute_world_z_acc_from_body(const IMUData &idata, float roll_deg, float pitch_deg) {
    // Convert to radians
    float r = roll_deg * (float)M_PI / 180.0f;
    float p = pitch_deg * (float)M_PI / 180.0f;

    // rotation matrix elements for body->world (Z component row)
    // R[2][0] = sin(p)
    // R[2][1] = -sin(r)*cos(p)
    // R[2][2] = cos(r)*cos(p)
    float R20 = sinf(p);
    float R21 = -sinf(r) * cosf(p);
    float R22 = cosf(r) * cosf(p);

    float ax = idata.acc[0];
    float ay = idata.acc[1];
    float az = idata.acc[2];

    float world_z = R20 * ax + R21 * ay + R22 * az; // world frame acceleration along down (m/s^2), includes gravity
    return world_z;
}

// ✅ **Update Barometer Readings (Matches Original Version)**
void updateBarometer() {
    unsigned long t_now = micros();
    float dt = (t_now - lastUpdate) / 1000000.0f;
    lastUpdate = t_now;
    if (dt < 0.002) return; // don't update too fast; preserve stability

    ms5611.read();
    float pressure = ms5611.getPressure();

    if (pressure <= 0 || isnan(pressure)) {
        Serial.println("Error: Invalid pressure reading!");
        return;
    }

    // --- One-time QNH set from GPS ---
    if (!qnhLocked) {
        if (gps.getSatellites() >= QNH_MIN_SATS_USED &&
            gps.getHDOP() <= QNH_MAX_HDOP &&
            !isnan(gps.getVDOP()) && gps.getVDOP() <= QNH_MAX_VDOP) {

            double gpsAlt = gps.getAltitude();
            if (gpsAlt > -1000 && gpsAlt < 10000) { // sanity check
                qnh = pressure / pow(1.0 - (gpsAlt / 44330.0), 5.255);
                qnhLocked = true;
                Serial.printf("[QNH] Locked from GPS: %.2f hPa (GPS Alt: %.1f m)\n", qnh, gpsAlt);
            }
        }
    }
    // --- end QNH gate ---

    pressureBuffer[pressureIndex] = pressure;
    pressureIndex = (pressureIndex + 1) % avgSamples;

    float pressureAvg = 0;
    for (int i = 0; i < avgSamples; i++) {
        pressureAvg += pressureBuffer[i];
    }
    pressureAvg /= avgSamples;

    altitude = (1.0f - pow(pressureAvg / qnh, 0.1903f)) * 44330.0f;

    if (useKalmanAltitude) {
        kalman_altitude = kalmanUpdate(kalman_altitude, altitude);
        altitude = kalman_altitude;
    }

    float v_measured = (altitude - h_prev) / dt;

    if (enableOutlierFilter) {
        static float lastValidVSpeed = 0;
        if (abs(v_measured - lastValidVSpeed) > outlierThreshold) {
            v_measured = lastValidVSpeed;
        } else {
            lastValidVSpeed = v_measured;
        }
    }

    // --- Optional IMU-based velocity integration and fusion ---
    float v_used = v_measured; // default to barometer
    if (useIMUFusion) {
        // Get roll/pitch from Attitude (we don't require yaw here)
        float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
        bool att_ok = Attitude::getEuler(roll_deg, pitch_deg, yaw_deg);

        // Only use IMU integration if Attitude is valid and tilt is reasonable
        float tilt = sqrtf(roll_deg * roll_deg + pitch_deg * pitch_deg);
        unsigned long now_us = micros();
        float dt_acc = (now_us - last_accel_ts) / 1000000.0f;
        if (dt_acc <= 0) dt_acc = 0.0f;
        last_accel_ts = now_us;

        if (att_ok && fabsf(tilt) <= MAX_ACCEPTED_TILT_DEG && dt_acc < 0.2f) {
            // compute world z acceleration (down) and remove gravity to get dynamic down acceleration
            float world_z = compute_world_z_acc_from_body(imu, roll_deg, pitch_deg); // includes gravity
            float a_dyn_down = world_z - G_const; // positive down = accelerating down faster than gravity

            // integrate -> velocity (down positive)
            v_accumulator += a_dyn_down * dt_acc;

            // now fuse: blended = alpha*baro + (1-alpha)*v_acc
            v_used = FUSE_ALPHA * v_measured + (1.0f - FUSE_ALPHA) * v_accumulator;
        } else {
            // when we can't rely on IMU, gently decay integrator towards baro to prevent runaway
            v_accumulator = (0.98f * v_accumulator) + (0.02f * v_measured);
            v_used = v_measured;
        }
    } // end useIMUFusion

    if (useKalmanVS) {
        kalman_vs = kalmanUpdate(kalman_vs, v_used);
        v_measured = kalman_vs;
    } else {
        v_measured = v_used;
    }

    // --- Smoothing options (same as before) ---
    if (useWMA) {
        float weightSum = 0, weightedVSum = 0;
        for (int i = 0; i < avgSamples; i++) {
            float weight = (i + 1);
            weightedVSum += vBuffer[i] * weight;
            weightSum += weight;
        }
        vSmoothed = weightedVSum / weightSum;
    } else if (useEMA) {
        float alpha = 0.2;
        vSmoothed = (alpha * v_measured) + ((1 - alpha) * vSmoothed);
    } else {
        vBuffer[vIndex] = v_measured;
        vIndex = (vIndex + 1) % avgSamples;

        float vAvg = 0;
        for (int i = 0; i < avgSamples; i++) {
            vAvg += vBuffer[i];
        }
        vSmoothed = vAvg / avgSamples;
    }

    vSmoothed = round(vSmoothed * 10.0f) / 10.0f;
    h_prev = altitude;
}
