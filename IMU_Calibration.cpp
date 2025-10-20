#include "IMU_Calibration.h"
#include <EEPROM.h>
#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include <lvgl.h>
#include <math.h>

// ---- EEPROM layout/versioning ----
static const int      EEPROM_ADDR = 0;
static const uint16_t MAGIC       = 0xA5C3;
static const uint8_t  VERSION     = 1;

struct IMUConfig {
    uint16_t magic;
    uint8_t  version;
    float    magOffset[3];
    float    magScale[3];
    float    accelOffset[3];
    float    accelScale[3];
    float    gyroBias[3];
};

static IMUConfig config;

// ---- Sensors ----
static MPU6050         mpu;
static QMC5883LCompass compass;

// ---- UI objects ----
static lv_obj_t* CalibMenuPanel     = nullptr;
static lv_obj_t* calib_status_label = nullptr;

// ---- EEPROM helpers ----
static bool s_eepromReady = false;

static void setCalibStatus(const char* msg) {
    if (calib_status_label) {
        lv_label_set_text(calib_status_label, msg);
        lv_timer_handler(); // allow immediate draw
    }
}
static void ensureEEPROM() {
    if (!s_eepromReady) {
        if (!EEPROM.begin(sizeof(IMUConfig))) {
            //Serial.println("[IMU] EEPROM.begin failed!");
        } else s_eepromReady = true;
    }
}
static bool eepromSave(const char* tag) {
    EEPROM.put(EEPROM_ADDR, config);
    bool ok = EEPROM.commit();
    //Serial.printf("[IMU] EEPROM commit (%s): %s\n", tag ? tag : "", ok ? "OK" : "FAILED");
    return ok;
}
static void dumpConfig(const char* tag) {
   // Serial.printf("[IMU] ---- Config Dump (%s) ----\n", tag ? tag : "");
    //Serial.printf(" magOffset: %.2f, %.2f, %.2f\n", config.magOffset[0], config.magOffset[1], config.magOffset[2]);
    //Serial.printf(" magScale : %.3f, %.3f, %.3f\n",  config.magScale[0],  config.magScale[1],  config.magScale[2]);
    //Serial.printf(" accOffset: %.3f, %.3f, %.3f\n", config.accelOffset[0], config.accelOffset[1], config.accelOffset[2]);
    //Serial.printf(" accScale : %.5f, %.5f, %.5f\n",  config.accelScale[0],  config.accelScale[1],  config.accelScale[2]);
    //Serial.printf(" gyroBias : %.3f, %.3f, %.3f\n",  config.gyroBias[0],   config.gyroBias[1],   config.gyroBias[2]);
    //Serial.println("-------------------------------");
}

// ---- RAW read helper (bypasses applyCalibration entirely) ----
static inline void readRawIMU(float acc[3], float gyro[3], float mag[3]) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Adjust if you changed full-scale ranges in begin()
    const float ACC_LSB_PER_G    = 16384.0f; // ±2g
    const float GYRO_LSB_PER_DPS = 131.0f;   // ±250 dps
    const float G = 9.81f;

    acc[0] = (ax / ACC_LSB_PER_G) * G;
    acc[1] = (ay / ACC_LSB_PER_G) * G;
    acc[2] = (az / ACC_LSB_PER_G) * G;

    gyro[0] = gx / GYRO_LSB_PER_DPS;
    gyro[1] = gy / GYRO_LSB_PER_DPS;
    gyro[2] = gz / GYRO_LSB_PER_DPS;

    compass.read();
    mag[0] = compass.getX();
    mag[1] = compass.getY();
    mag[2] = compass.getZ();
}

// ---- Public API ----
void IMU_Calibration::begin() {
    ensureEEPROM();
    Wire.begin();
    mpu.initialize();
    compass.init();

    // Stabilize outputs: DLPF and sample rate
    mpu.setDLPFMode(4); // ~20 Hz bandwidth
    mpu.setRate(4);     // 1 kHz/(1+4) = 200 Hz internal

    // Ensure full-scale ranges match the divisors used in readRawIMU()
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    EEPROM.get(EEPROM_ADDR, config);
    if (config.magic != MAGIC || config.version != VERSION) {
        memset(&config, 0, sizeof(config));
        for (int i=0; i<3; i++) {
            config.magScale[i]   = 1.0f;
            config.accelScale[i] = 1.0f;
        }
        config.magic = MAGIC;
        config.version = VERSION;
        eepromSave("seed-defaults");
    }
    dumpConfig("begin");
}

static bool s_suspendApplyDuringFused = false;

void IMU_Calibration::applyCalibration(IMUData &data) {
    if (s_suspendApplyDuringFused) return; // bypass during fused capture
    for (int i=0; i<3; i++) {
        data.mag[i]  = (data.mag[i]  - config.magOffset[i])   * config.magScale[i];
        data.acc[i]  = (data.acc[i]  - config.accelOffset[i]) * config.accelScale[i];
        data.gyro[i] = (data.gyro[i] - config.gyroBias[i]);
    }
}

// ---- Mag-only calibration (unchanged) ----
static bool          calibratingMag = false;
static unsigned long calStart       = 0;
static long          magMin[3], magMax[3];

void IMU_Calibration::startMagCalibration() {
    calibratingMag = true;
    calStart = millis();
    for (int i=0; i<3; i++) { magMin[i]=32767; magMax[i]=-32768; }
    setCalibStatus("Rotate device slowly for 30s (mag)...");
    //Serial.println("[IMU] *** MAG CALIBRATION STARTED ***");
}

// ---- Fused calibration state machine (ungated coverage + gated validation) ----
enum class FusedState : uint8_t { Idle=0, GyroBias, Rotate, Finalize };
static FusedState    fusedState        = FusedState::Idle;
static unsigned long stateStartMs      = 0;
static unsigned long rotateDurationMs  = 70000; // 70s for better coverage
static IMUConfig     configBackup;

static const float G = 9.81f;

// Coverage (ALL samples)
static float accMinAll[3], accMaxAll[3];
static float magMinF[3],   magMaxF[3];
static bool  accSeenPos[3], accSeenNeg[3];

// Fit/validation (GATED only)
static float accMinG[3], accMaxG[3];

// Gating thresholds (quasi-static) — loosened for field
static const float GATE_GMAG_TOL = 1.4f;   // accept if | |a| - g | < 2.5 m/s^2
static const float GATE_GYRO_DPS = 80.0f; // accept if gyro Euclidean mag < 140 dps

// Validator thresholds (field-friendly)
static const float ACC_SPAN_MIN        = 2.0f;   // min half-span per axis (gated)
static const float ACC_POLARITY_THRESH = 0.5f * G; // for coverage only (all samples)
static const int   VALID_N_MIN         = 40;     // enough quasi-static points
static const float VALID_RMS_MAX       = 1.2f;
static const float VALID_MEAN_ABS_MAX  = 0.8f;

// Buffer for validation (RAW, gated only)
struct AccRing {
    static const int N = 500;
    int idx = 0, fill = 0;
    float v[N][3];
    void clear(){ idx=0; fill=0; }
    void push(const float a[3]) {
        v[idx][0]=a[0]; v[idx][1]=a[1]; v[idx][2]=a[2];
        idx = (idx + 1) % N; if (fill < N) fill++;
    }
} s_accRing;

static void fusedEnter(FusedState s) {
    fusedState   = s;
    stateStartMs = millis();
}

void IMU_Calibration::startFusedCalibration(uint32_t rotationMs) {
    // Reset trackers
    for (int i=0;i<3;i++) {
        accMinAll[i]=  1e9f; accMaxAll[i]= -1e9f;
        accMinG[i]  =  1e9f; accMaxG[i]  = -1e9f;
        magMinF[i]= 1e9f;    magMaxF[i]= -1e9f;
        accSeenPos[i]=accSeenNeg[i]=false;
    }
    s_accRing.clear();

    rotateDurationMs = rotationMs ? rotationMs : rotateDurationMs;
    configBackup     = config;

    s_suspendApplyDuringFused = true; // ensure all reads are RAW during fused

    setCalibStatus("Hold still for gyro bias...");
    //Serial.println("[IMU] *** FUSED CALIBRATION: GYRO BIAS PHASE ***");
    fusedEnter(FusedState::GyroBias);
}

void IMU_Calibration::startGyroBiasCalibration() {
    // Standalone manual bias capture (blocking, short)
    setCalibStatus("Hold still (gyro bias)...");
    //Serial.println("[IMU] *** GYRO BIAS CALIBRATION STARTED ***");
    delay(200); // allow UI to draw
    float sum[3] = {0};
    const int samples = 200;
    for (int i = 0; i < samples; i++) {
        float a[3], g[3], m[3];
        readRawIMU(a,g,m);
        sum[0]+=g[0]; sum[1]+=g[1]; sum[2]+=g[2];
        delay(5);
    }
    for (int i=0;i<3;i++) config.gyroBias[i] = sum[i] / samples;
    eepromSave("gyro");
    dumpConfig("gyro-complete");
    setCalibStatus("Gyro bias saved");
    //Serial.println("[IMU] *** GYRO BIAS CALIBRATION COMPLETE ***");
}

void IMU_Calibration::resetCalibration() {
    memset(&config, 0, sizeof(config));
    for (int i=0;i<3;i++) {
        config.magScale[i]   = 1.0f;
        config.accelScale[i] = 1.0f;
    }
    config.magic = MAGIC; config.version = VERSION;
    eepromSave("reset");
    dumpConfig("reset");
    setCalibStatus("Calibration reset");
    //Serial.println("[IMU] Calibration reset");
}

bool IMU_Calibration::isCalibrating() {
    return calibratingMag || fusedState != FusedState::Idle;
}

// ---- Update (drives mag-only and fused state machine) ----
void IMU_Calibration::update() {
    // Mag-only
    if (calibratingMag) {
        compass.read();
        long mx = compass.getX(), my = compass.getY(), mz = compass.getZ();
        if (mx < magMin[0]) magMin[0] = mx; if (mx > magMax[0]) magMax[0] = mx;
        if (my < magMin[1]) magMin[1] = my; if (my > magMax[1]) magMax[1] = my;
        if (mz < magMin[2]) magMin[2] = mz; if (mz > magMax[2]) magMax[2] = mz;

        if (millis() - calStart >= 30000) {
            calibratingMag = false;
            float halfSpan[3], ref=0;
            for (int i=0;i<3;i++) {
                config.magOffset[i] = (magMax[i] + magMin[i]) / 2.0f;
                halfSpan[i] = (magMax[i] - magMin[i]) / 2.0f;
                if (halfSpan[i] <= 0) halfSpan[i] = 1;
                ref += halfSpan[i];
            }
            ref /= 3.0f;
            for (int i=0;i<3;i++) config.magScale[i] = ref / halfSpan[i];
            eepromSave("mag");
            dumpConfig("mag-complete");
            setCalibStatus("Mag calib COMPLETE");
            //Serial.println("[IMU] *** MAG CALIBRATION COMPLETE ***");
        }
    }

    // Fused state machine
    switch (fusedState) {
        case FusedState::Idle:
            return;

        case FusedState::GyroBias: {
            const unsigned long biasDelayMs   = 600;   // show message before sampling
            const unsigned long biasSampleMs  = 1800;  // sampling window (~1.8s)
            static bool startedSampling = false;
            static float sum[3] = {0,0,0};
            static uint16_t count = 0;

            unsigned long elapsed = millis() - stateStartMs;

            if (!startedSampling && elapsed >= biasDelayMs) {
                startedSampling = true;
                sum[0]=sum[1]=sum[2]=0; count=0;
                //Serial.println("[CAL] Gyro bias sampling...");
            }

            if (startedSampling && elapsed < (biasDelayMs + biasSampleMs)) {
                float a[3], g[3], m[3];
                readRawIMU(a,g,m);
                sum[0]+=g[0]; sum[1]+=g[1]; sum[2]+=g[2];
                count++;
                break;
            }

            if (elapsed >= (biasDelayMs + biasSampleMs)) {
                if (count > 0) {
                    config.gyroBias[0] = sum[0] / count;
                    config.gyroBias[1] = sum[1] / count;
                    config.gyroBias[2] = sum[2] / count;
                }
                //Serial.printf("[CAL] Gyro bias: %.3f, %.3f, %.3f (n=%u)\n",
                              //config.gyroBias[0], config.gyroBias[1], config.gyroBias[2], count);

                for (int i=0;i<3;i++) {
                    accMinAll[i]=  1e9f; accMaxAll[i]= -1e9f;
                    accMinG[i]  =  1e9f; accMaxG[i]  = -1e9f;
                    magMinF[i]= 1e9f;   magMaxF[i]= -1e9f;
                    accSeenPos[i]=accSeenNeg[i]=false;
                }
                s_accRing.clear();

                setCalibStatus("Rotate slowly in all directions...");
                //Serial.println("[IMU] *** FUSED CALIBRATION: ROTATION PHASE ***");
                fusedEnter(FusedState::Rotate);
            }
            break;
        }

        case FusedState::Rotate: {
            float a[3], g[3], m[3];
            readRawIMU(a,g,m);

            // Always update coverage (ALL samples)
            for (int i=0;i<3;i++) {
                if (a[i] < accMinAll[i]) accMinAll[i]=a[i];
                if (a[i] > accMaxAll[i]) accMaxAll[i]=a[i];
                if (m[i] < magMinF[i])   magMinF[i]=m[i];
                if (m[i] > magMaxF[i])   magMaxF[i]=m[i];
                if (a[i] >  ACC_POLARITY_THRESH) accSeenPos[i]=true;
                if (a[i] < -ACC_POLARITY_THRESH) accSeenNeg[i]=true;
            }

            // Gate only the validation and fit buffers (quasi-static)
            const float gmag = sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
            const float dG   = fabsf(gmag - G);
            const float gy   = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]); // Euclidean mag
            const bool accept = (dG <= GATE_GMAG_TOL) && (gy < GATE_GYRO_DPS);
            if (accept) {
                s_accRing.push(a);
                for (int i=0;i<3;i++) {
                    if (a[i] < accMinG[i]) accMinG[i]=a[i];
                    if (a[i] > accMaxG[i]) accMaxG[i]=a[i];
                }
            }

            // 1 Hz progress
            static unsigned long lastLog=0;
            if (millis()-lastLog>1000) {
                lastLog = millis();
                //Serial.printf("[CAL] g=%.2f dG=%.2f gyro=%.1f %s N=%d\n",
                              //gmag, dG, gy, accept?"OK":"skip", s_accRing.fill);
            }

            // Countdown
            unsigned long elapsed = millis() - stateStartMs;
            unsigned long remaining = (rotateDurationMs > elapsed) ? (rotateDurationMs - elapsed)/1000 : 0;
            static unsigned long lastSec = 999;
            if (remaining != lastSec) {
                lastSec = remaining;
                char buf[48];
                snprintf(buf, sizeof(buf), "Fused calib: %lus left", (unsigned long)remaining);
                setCalibStatus(buf);
                //Serial.println(buf);
            }

            if (elapsed >= rotateDurationMs) {
                // Coverage snapshot (ALL + GATED)
                for (int i=0;i<3;i++) {
                    //Serial.printf("[CAL] ALL ACC axis %d min/max: [%.2f, %.2f] span=%.2f\n",
                                  //i, accMinAll[i], accMaxAll[i], accMaxAll[i]-accMinAll[i]);
                }
                for (int i=0;i<3;i++) {
                    //Serial.printf("[CAL] GATED ACC axis %d min/max: [%.2f, %.2f] span=%.2f\n",
                                  //i, accMinG[i], accMaxG[i], accMaxG[i]-accMinG[i]);
                }
                for (int i=0;i<3;i++) {
                    //Serial.printf("[CAL] MAG axis %d min/max: [%.0f, %.0f] span=%.0f\n",
                                  //i, magMinF[i], magMaxF[i], magMaxF[i]-magMinF[i]);
                }
                setCalibStatus("Validating…");
                fusedEnter(FusedState::Finalize);
            }
            break;
        }

        case FusedState::Finalize: {
            // Compute accel offset/scale using GATED min/max (not ALL)
            bool accOK = true;
            for (int i=0;i<3;i++) {
                const float halfSpan = 0.5f * (accMaxG[i] - accMinG[i]);
                const float mid      = 0.5f * (accMaxG[i] + accMinG[i]);

                if (halfSpan < ACC_SPAN_MIN || !(accSeenPos[i] && accSeenNeg[i])) {
                    //Serial.printf("[FAIL] Axis %d insufficient span/coverage (gated halfSpan=%.2f, +%d -%d)\n",
                                  //i, halfSpan, accSeenPos[i], accSeenNeg[i]);
                    accOK = false;
                } else {
                    config.accelOffset[i] = mid;
                    config.accelScale[i]  = (halfSpan > 1e-6f) ? (G / halfSpan) : 1.0f;
                }
            }

            // Magnetometer min-max equalization (ALL samples)
            float mHalf[3]; float mRef=0;
            for (int i=0;i<3;i++) {
                const float h = 0.5f * (magMaxF[i] - magMinF[i]);
                mHalf[i] = (h > 1.0f) ? h : 1.0f;
                config.magOffset[i] = 0.5f * (magMaxF[i] + magMinF[i]);
                mRef += mHalf[i];
            }
            mRef = mRef / 3.0f;
            for (int i=0;i<3;i++) config.magScale[i] = mRef / mHalf[i];

            // Validation on gated RAW samples
            float rms=0, mean=0; int N=s_accRing.fill;
            for (int k=0;k<N;k++) {
                float ax = (s_accRing.v[k][0] - config.accelOffset[0]) * config.accelScale[0];
                float ay = (s_accRing.v[k][1] - config.accelOffset[1]) * config.accelScale[1];
                float az = (s_accRing.v[k][2] - config.accelOffset[2]) * config.accelScale[2];
                float e  = sqrtf(ax*ax+ay*ay+az*az) - G;
                mean += e; rms += e*e;
            }
            if (N>0) { mean/=N; rms = sqrtf(rms/N); }

            //Serial.printf("[CAL] ACC validate (gated): N=%d, mean=%.3f, rms=%.3f\n", N, mean, rms);

            const bool pass = accOK && N > VALID_N_MIN && rms < VALID_RMS_MAX && fabsf(mean) < VALID_MEAN_ABS_MAX;

            if (!pass) {
                if (!accOK) //Serial.println("[FAIL] Accel coverage/span check failed");
                if (N <= VALID_N_MIN) //Serial.printf("[FAIL] Validation sample count too low: %d\n", N);
                if (rms >= VALID_RMS_MAX) //Serial.printf("[FAIL] RMS error too high: %.3f\n", rms);
                if (fabsf(mean) >= VALID_MEAN_ABS_MAX) //Serial.printf("[FAIL] Mean error too high: %.3f\n", mean);

                //Serial.println("[IMU] *** FUSED CALIBRATION FAILED – REVERTING ***");
                config = configBackup;
                setCalibStatus("Fused calib FAILED");
            } else {
                eepromSave("fused");
                dumpConfig("fused-complete");
                setCalibStatus("Fused calib COMPLETE");
                //Serial.println("[IMU] *** FUSED CALIBRATION COMPLETE ***");
            }

            // Raw vs calibrated sample dump (from RAW, using current config)
            //Serial.println("[IMU] Sample raw vs calibrated (acc only):");
            int show = (N < 5) ? N : 5;
            for (int k = 0; k < show; k++) {
                float rx = s_accRing.v[k][0];
                float ry = s_accRing.v[k][1];
                float rz = s_accRing.v[k][2];
                float cx = (rx - config.accelOffset[0]) * config.accelScale[0];
                float cy = (ry - config.accelOffset[1]) * config.accelScale[1];
                float cz = (rz - config.accelOffset[2]) * config.accelScale[2];
                //Serial.printf("RAW: [%6.2f %6.2f %6.2f]  CAL: [%6.2f %6.2f %6.2f]\n",
  //                            rx, ry, rz, cx, cy, cz);
            }

            s_suspendApplyDuringFused = false; // restore normal processing
            fusedEnter(FusedState::Idle);
            break;
        }
    } // switch
}

// ---- Calibration menu ----
void IMU_Calibration::showCalibMenu() {
    if (CalibMenuPanel) {
        lv_obj_clear_flag(CalibMenuPanel, LV_OBJ_FLAG_HIDDEN);
        setCalibStatus("Ready.");
        return;
    }

    CalibMenuPanel = lv_obj_create(lv_scr_act());
    lv_obj_set_size(CalibMenuPanel, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_bg_color(CalibMenuPanel, lv_color_black(), 0);
    lv_obj_set_style_text_color(CalibMenuPanel, lv_color_white(), 0);
    lv_obj_clear_flag(CalibMenuPanel, LV_OBJ_FLAG_SCROLLABLE);

    calib_status_label = lv_label_create(CalibMenuPanel);
    lv_obj_set_pos(calib_status_label, 10, 8);
    lv_label_set_text(calib_status_label, "Calibration Menu");
    setCalibStatus("Ready.");

    int y = 60;
    auto add_btn = [&](const char* label, lv_event_cb_t cb) {
        lv_obj_t* btn = lv_btn_create(CalibMenuPanel);
        lv_obj_set_size(btn, 160, 44);
        lv_obj_set_pos(btn, 10, y);
        lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, label);
        lv_obj_center(lbl);
        y += 52;
    };

    add_btn("Fused Calib", [](lv_event_t* e) {
        startFusedCalibration(70000); // 70s rotation for better coverage
    });
    add_btn("Mag Calib", [](lv_event_t* e) { startMagCalibration(); });
    add_btn("Gyro Bias", [](lv_event_t* e) { startGyroBiasCalibration(); });

    // Exit
    lv_obj_t* back_btn = lv_btn_create(CalibMenuPanel);
    lv_obj_set_size(back_btn, 80, 34);
    lv_obj_set_pos(back_btn, 190, 200);
    lv_obj_add_event_cb(back_btn, [](lv_event_t* e) {
        lv_obj_add_flag(CalibMenuPanel, LV_OBJ_FLAG_HIDDEN);
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_t* back_lbl = lv_label_create(back_btn);
    lv_label_set_text(back_lbl, "Back");
    lv_obj_center(back_lbl);
}
