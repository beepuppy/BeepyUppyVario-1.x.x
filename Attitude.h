#pragma once
#include "IMU.h"

// -------------------- Attitude class --------------------
class Attitude {
public:
    // Struct of tunable parameters.
    // These can be optionally set before calling init().
    // If left unset (0), defaults will be applied automatically.
    struct Settings {
        // --- Filters ---
        float accel_alpha = 0.25f;         // Low-pass for accelerometer (0..1)
        float mag_alpha   = 0.20f;         // Low-pass for magnetometer (0..1)

        // --- Fusion weights ---
        float mag_correction_gain = 0.6f; // Not used in all builds, safe default
        float declination_deg     = -1.0f;  // Local magnetic declination (+E)

        // --- Trust gates (added back for full tilt-comp compatibility) ---
        float accel_g_tolerance   = 1.5f;  // Acceptable |acc|-9.81 difference
        float gyro_norm_gate      = 120.0f; // Max allowed total gyro activity (deg/s)
        float mag_norm_frac_lo    = 0.7f;  // Minimum acceptable magnetic field ratio
        float mag_norm_frac_hi    = 1.3f;  // Maximum acceptable magnetic field ratio

        // --- Complementary filter weights ---
        float rp_gyro_weight      = 0.94f; // Roll/pitch gyro fusion weight
        float yaw_gyro_weight     = 0.95f; // Yaw gyro fusion weight
    };

    // Initialise the attitude filter
    static void init(const Settings& s);

    // Set magnetic declination (in degrees)
    static void setDeclination(float declination_deg);

    // Update attitude estimate with new IMU data and time delta (seconds)
    static void update(const IMUData& d, float dt_seconds);

    // Retrieve roll/pitch/yaw (degrees). Returns true if valid.
    static bool getEuler(float& roll_deg, float& pitch_deg, float& yaw_deg);

    // Get continuous (unwrapped) yaw angle (deg, can exceed 360)
    static float getYawUnwrappedDeg();

    // Trust flags (for diagnostics or display)
    static bool accelTrusted();
    static bool magTrusted();

    // Get norms (magnitudes) of accel and mag (for debugging)
    static void getNorms(float& a_norm_out, float& m_norm_out);
};
