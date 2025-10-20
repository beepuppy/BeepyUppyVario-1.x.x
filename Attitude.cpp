#include "Attitude.h"
#include "IMU.h"
#include <math.h>

// -------------------- Mounting matrix --------------------
// Sensor -> Body rotation (X forward, Y right, Z down)
// Derived from 3-pose mounting solve (commissioned 2025-09-11).
static const float R_sb[3][3] = {
  {-0.049f,  0.999f, -0.012f},
  { 0.996f,  0.092f,  0.003f},
  { 0.010f,  0.058f, -0.998f}
};

static inline void toBody(const float v_s[3], float v_b[3]) {
  v_b[0] = R_sb[0][0]*v_s[0] + R_sb[0][1]*v_s[1] + R_sb[0][2]*v_s[2];
  v_b[1] = R_sb[1][0]*v_s[0] + R_sb[1][1]*v_s[1] + R_sb[1][2]*v_s[2];
  v_b[2] = R_sb[2][0]*v_s[0] + R_sb[2][1]*v_s[1] + R_sb[2][2]*v_s[2];
}

// -------------------- Internal state --------------------
namespace {
  Attitude::Settings S;

  // Filtered, mounted vectors
  float acc_b[3]  = {0,0,9.81f};
  float mag_b[3]  = {0,0,0};
  float gyro_b[3] = {0,0,0}; // deg/s

  // Euler (deg)
  float roll_d  = 0.0f;
  float pitch_d = 0.0f;
  float yaw_d   = 0.0f;     // 0..360
  float yaw_unw = 0.0f;     // unwrapped

  // Trust flags
  bool use_accel_this_frame = true;
  bool use_mag_this_frame   = true;

  // Nominal magnitudes for gating
  const float G = 9.81f;
  float M_nominal = 600.0f; // learned on first few seconds

  bool initialized = false;
  unsigned long start_ms = 0;

  // temp locals used during update
  float a_b_now[3], m_b_now[3], g_b_now[3];

  static inline float rad2deg(float r){ return r * 57.2957795f; }
  static inline float deg2rad(float d){ return d * 0.01745329252f; }

  static inline float wrap360(float a){
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
  }
  static inline float wrap_pm180(float a){
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
  }

  static inline float vnorm(const float v[3]){
    return sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
  }

  static inline void vcopy(const float a[3], float b[3]){
    b[0]=a[0]; b[1]=a[1]; b[2]=a[2];
  }

  // LPF (alpha is 0..1: 1=instant, 0=no change)
  static inline void lpf3(float alpha, const float in[3], float state[3]) {
    // stable form: state = alpha*in + (1-alpha)*state
    state[0] = alpha*in[0] + (1.0f-alpha)*state[0];
    state[1] = alpha*in[1] + (1.0f-alpha)*state[1];
    state[2] = alpha*in[2] + (1.0f-alpha)*state[2];
  }

  // Compute roll/pitch from accelerometer (body frame).
  static void accelToRollPitch(const float a_b[3], float& roll_out_deg, float& pitch_out_deg) {
    float ax=a_b[0], ay=a_b[1], az=a_b[2];
    float roll  = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
    roll_out_deg  = rad2deg(roll);
    pitch_out_deg = rad2deg(pitch);
  }

  // Tilt-compensated yaw from mag using current roll/pitch
static float tiltCompYawDeg(const float m_b[3], float roll_deg, float pitch_deg) {
  float cr = cosf(deg2rad(roll_deg));
  float sr = sinf(deg2rad(roll_deg));
  float cp = cosf(deg2rad(pitch_deg));
  float sp = sinf(deg2rad(pitch_deg));

  // Body axes: X forward, Y right, Z down
  float mx = m_b[0], my = m_b[1], mz = m_b[2];

  // Adjusted tilt comp for your mounting (pitch/roll relative to body)
  float mxh = mx * cp - mz * sp;             // corrected sign for pitch
  float myh = mx * sr * sp + my * cr + mz * sr * cp;

  float yaw = atan2f(-myh, mxh); // heading, east-positive
  return wrap360(rad2deg(yaw));
}


  static void seedFromAccelMag(const float a_b_now[3], const float m_b_now[3]) {
    accelToRollPitch(a_b_now, roll_d, pitch_d);
    yaw_d = tiltCompYawDeg(m_b_now, roll_d, pitch_d);
    yaw_unw = yaw_d;
  }
}

// ---------------- PUBLIC API ----------------

void Attitude::init(const Settings& s) {
  S = s;
  initialized = false;
  start_ms = millis();
  acc_b[0]=0; acc_b[1]=0; acc_b[2]=G;
  mag_b[0]=0; mag_b[1]=1; mag_b[2]=0;
  gyro_b[0]=gyro_b[1]=gyro_b[2]=0;
  roll_d = pitch_d = yaw_d = yaw_unw = 0.0f;
  M_nominal = 600.0f;
}

void Attitude::setDeclination(float declination_deg) {
  S.declination_deg = declination_deg;
}

void Attitude::update(const IMUData& d, float dt) {
  if (dt <= 0.0f) return;

  // 1) Mount all vectors into body frame (sensor->body)
  toBody(d.acc,  a_b_now);
// Re-map magnetometer axes before rotation into body frame
float mag_mapped_s[3];
mag_mapped_s[0] =  d.mag[1];   // sensor Y → body X
mag_mapped_s[1] = -d.mag[0];   // -sensor X → body Y
mag_mapped_s[2] =  d.mag[2];   // sensor Z → body Z
toBody(mag_mapped_s, m_b_now);
  toBody(d.gyro, g_b_now); // deg/s

  // 2) Low-pass accel & mag (gyro used as-is for complementary)
  // Ensure alpha in sensible range
  float a_alpha = S.accel_alpha;
  if (a_alpha < 0.001f) a_alpha = 0.001f;
  if (a_alpha > 1.0f) a_alpha = 1.0f;
  float m_alpha = S.mag_alpha;
  if (m_alpha < 0.001f) m_alpha = 0.001f;
  if (m_alpha > 1.0f) m_alpha = 1.0f;

  lpf3(a_alpha, a_b_now, acc_b);
  lpf3(m_alpha, m_b_now, mag_b);
  vcopy(g_b_now, gyro_b);

//#ifdef ATTITUDE_DEBUG
 // Serial.print("LPF accel: "); Serial.print(acc_b[0]); Serial.print(","); Serial.print(acc_b[1]); Serial.print(","); Serial.println(acc_b[2]);
  //Serial.print("LPF mag:   "); Serial.print(mag_b[0]); Serial.print(","); Serial.print(mag_b[1]); Serial.print(","); Serial.println(mag_b[2]);
  //Serial.print("gyro_b:    "); Serial.print(gyro_b[0]); Serial.print(","); Serial.print(gyro_b[1]); Serial.print(","); Serial.println(gyro_b[2]);
//#endif

  // 3) Learn nominal |m| over the first few seconds for gating
  if (!initialized) {
    static int warm = 0;
    const int WARM_N = 200; // ~1–2s depending on loop
    float mnorm = vnorm(mag_b);
    if (M_nominal == 600.0f) M_nominal = mnorm; // first sample
    M_nominal = 0.98f * M_nominal + 0.02f * mnorm;
    if (++warm > WARM_N) {
      seedFromAccelMag(acc_b, mag_b);
      initialized = true;
//#ifdef ATTITUDE_DEBUG
      //Serial.print("Initialized. M_nominal="); Serial.println(M_nominal);
//#endif
    }
    return; // don't run fusion until warm
  }

  // 4) Quality gates
  float a_norm = vnorm(acc_b);
  float m_norm = vnorm(mag_b);
  float gyro_norm = fabsf(gyro_b[0]) + fabsf(gyro_b[1]) + fabsf(gyro_b[2]);

  use_accel_this_frame =
    (fabsf(a_norm - G) <= S.accel_g_tolerance) &&
    (gyro_norm < S.gyro_norm_gate);

  float mfrac = (M_nominal > 1e-3f) ? (m_norm / M_nominal) : 1.0f;
  use_mag_this_frame =
    (mfrac >= S.mag_norm_frac_lo) &&
    (mfrac <= S.mag_norm_frac_hi);

//#ifdef ATTITUDE_DEBUG
  //Serial.print("a_norm="); Serial.print(a_norm);
  //Serial.print(" m_norm="); Serial.print(m_norm);
  //Serial.print(" mfrac="); Serial.print(mfrac);
  //Serial.print(" use_accel="); Serial.print(use_accel_this_frame);
  //Serial.print(" use_mag="); Serial.println(use_mag_this_frame);
//#endif

  // 5) Compute accel-derived rp and mag-derived yaw
  float rp_roll  = roll_d;
  float rp_pitch = pitch_d;
  if (use_accel_this_frame) {
    accelToRollPitch(acc_b, rp_roll, rp_pitch);
  }

  float mag_yaw = yaw_d;
  if (use_mag_this_frame) {
    mag_yaw = tiltCompYawDeg(mag_b, rp_roll, rp_pitch);
    mag_yaw = wrap360(mag_yaw + S.declination_deg);
  }

  // 6) Complementary fusion with gyro integration
  // Body rates: p=gyro_b[0] (roll rate), q=gyro_b[1] (pitch rate), r=gyro_b[2] (yaw rate)
  float roll_gyro  = roll_d  + gyro_b[0] * dt;
  float pitch_gyro = pitch_d + gyro_b[1] * dt;
  float yaw_gyro   = yaw_d   - gyro_b[2] * dt; // note sign convention as in older code

  // Fuse roll/pitch
  float wg = S.rp_gyro_weight;
  float wc = 1.0f - wg;
  if (use_accel_this_frame) {
    roll_d  = wg * roll_gyro  + wc * rp_roll;
    pitch_d = wg * pitch_gyro + wc * rp_pitch;
  } else {
    roll_d  = roll_gyro;
    pitch_d = pitch_gyro;
  }

  // Fuse yaw
  float wy = S.yaw_gyro_weight;
  float wyc = 1.0f - wy;

  float mag_err = wrap_pm180(mag_yaw - yaw_d);
  float yaw_mag_unwrapped = yaw_d + mag_err;

  if (use_mag_this_frame) {
    float yaw_fused = wy * yaw_gyro + wyc * yaw_mag_unwrapped;
    float yaw_prev = yaw_d;
    yaw_d = wrap360(yaw_fused);
    float dy = wrap_pm180(yaw_d - yaw_prev);
    yaw_unw += dy;
  } else {
    float yaw_prev = yaw_d;
    yaw_d = wrap360(yaw_gyro);
    float dy = wrap_pm180(yaw_d - yaw_prev);
    yaw_unw += dy;
  }
}

bool Attitude::getEuler(float& roll_deg, float& pitch_deg, float& yaw_deg) {
  if (!initialized) return false;
  roll_deg  = roll_d;
  pitch_deg = pitch_d;
  yaw_deg   = yaw_d;
  return true;
}

float Attitude::getYawUnwrappedDeg() {
  return yaw_unw;
}

bool Attitude::accelTrusted() { return use_accel_this_frame; }
bool Attitude::magTrusted()   { return use_mag_this_frame;   }

void Attitude::getNorms(float& a_norm, float& m_norm) {
  a_norm = vnorm(acc_b);
  m_norm = vnorm(mag_b);
}
