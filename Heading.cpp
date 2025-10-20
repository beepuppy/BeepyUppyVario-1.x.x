#include "Heading.h"
#include "gps.h"
#include "Attitude.h"
#include <Arduino.h>
#include <math.h>

extern GPSHandler gps;

// --- Config ---
// Blend IMU <-> GPS based on speed (m/s). Below 2 m/s use IMU, above 5 m/s use GPS.
static const float BLEND_V_MIN = 2.0f;  // m/s
static const float BLEND_V_MAX = 5.0f;  // m/s

// Smoothing of the fused heading (on unit circle)
static const float SMOOTH_ALPHA = 0.7f;

// If both GPS invalid and IMU unavailable, hold last heading for this long
static const uint32_t HOLD_MS = 4000;

// --- State ---
static float headingDegrees = NAN;         // fused (0..360)
static float headingUnwrappedDeg = NAN;    // fused continuous
static float s_prevNormDeg = NAN;
static float headingOffsetDeg = 0.0f;  // user-defined offset in degrees

static struct {
  float sin_acc = 0.0f;
  float cos_acc = 1.0f;
  bool primed = false;
} s_hf;

static uint32_t lastValidMs = 0;
static float lastValidDeg = NAN;

// --- Utils ---
static inline float norm360(float d) {
  while (d < 0.0f) d += 360.0f;
  while (d >= 360.0f) d -= 360.0f;
  return d;
}
static inline float wrap_pm180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}
// Smooth heading on unit circle
static float hf_update(float hdg_deg_cw) {
  float rad = hdg_deg_cw * (float)M_PI / 180.0f;
  float sN = sinf(rad), cN = cosf(rad);
  if (!s_hf.primed) {
    s_hf.sin_acc = sN;
    s_hf.cos_acc = cN;
    s_hf.primed = true;
  } else {
    s_hf.sin_acc = SMOOTH_ALPHA * s_hf.sin_acc + (1.0f - SMOOTH_ALPHA) * sN;
    s_hf.cos_acc = SMOOTH_ALPHA * s_hf.cos_acc + (1.0f - SMOOTH_ALPHA) * cN;
  }
  float out = atan2f(s_hf.sin_acc, s_hf.cos_acc) * 180.0f / (float)M_PI;
  return norm360(out);
}

void initHeadingSensors() {
  headingDegrees = NAN;
  headingUnwrappedDeg = NAN;
  s_prevNormDeg = NAN;
  s_hf.sin_acc = 0.0f;
  s_hf.cos_acc = 1.0f;
  s_hf.primed  = false;
  lastValidMs = 0;
  lastValidDeg = NAN;
  Serial.println("[HDG] Heading system initialized (IMU+GPS fused)");
}

void setHeadingOffset(float offsetDeg) {
  headingOffsetDeg = offsetDeg;
}


void updateHeading() {
  uint32_t now = millis();

  // Read IMU yaw (0..360). Return false until Attitude is initialized.
  float roll, pitch, yaw_imu_deg = NAN;
  bool imu_ok = Attitude::getEuler(roll, pitch, yaw_imu_deg);

  // Read GPS course (deg, 0..360) and speed (km/h from gps)
  double course_gps_deg = gps.getCourse();
  double speed_kmh = gps.getSpeed();
  bool gps_valid = gps.isValid() && course_gps_deg >= 0.0;

  // Compute blend factor k in [0,1] based on speed (m/s)
  float speed_mps = (float)speed_kmh * (1.0f / 3.6f);
  float k = 0.0f;
  if (gps_valid) {
    if      (speed_mps <= BLEND_V_MIN) k = 0.0f;        // rely on IMU
    else if (speed_mps >= BLEND_V_MAX) k = 1.0f;        // rely on GPS
    else                               k = (speed_mps - BLEND_V_MIN) / (BLEND_V_MAX - BLEND_V_MIN);
  }

  // Choose the fused heading
  float fused_deg = NAN;

  if (imu_ok && gps_valid) {
    float imu = norm360(yaw_imu_deg);
    float gps = norm360((float)course_gps_deg);

    // Blend on the short arc: fused = IMU + k * (GPS - IMU)
    float diff = wrap_pm180(gps - imu);
    fused_deg = norm360(imu + k * diff);

  } else if (imu_ok && !gps_valid) {
    // IMU only
    fused_deg = norm360(yaw_imu_deg);

  } else if (!imu_ok && gps_valid) {
    // GPS only (e.g., just after boot while Attitude warms up)
    fused_deg = norm360((float)course_gps_deg);

  } else {
    // Neither available: hold last for a short period
    uint32_t dt = now - lastValidMs;
    if (isnan(lastValidDeg) || dt > HOLD_MS) {
      headingDegrees = NAN;
      return;
    } else {
      fused_deg = lastValidDeg;
    }
  }

  // Smooth fused heading (keeps UI pleasant without lagging too much)
  float filtered = hf_update(fused_deg);

  // Publish + bookkeeping
  headingDegrees = filtered;
  lastValidDeg = filtered;
  lastValidMs = now;

  // Unwrapped fused heading
  if (isnan(s_prevNormDeg)) {
    s_prevNormDeg = headingDegrees;
    headingUnwrappedDeg = headingDegrees;
  } else {
    float delta = wrap_pm180(headingDegrees - s_prevNormDeg);
    headingUnwrappedDeg += delta;
    s_prevNormDeg = headingDegrees;
  }
}

float getHeadingDegrees() {
  return isnan(headingDegrees) ? 0.0f : norm360(headingDegrees + headingOffsetDeg);
}

float getHeadingUnwrappedDegrees() {
  return isnan(headingUnwrappedDeg) ? 0.0f : headingUnwrappedDeg + headingOffsetDeg;
}

