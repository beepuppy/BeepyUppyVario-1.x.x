#include "battery.h"
#include <Preferences.h>

namespace battery {
  static const uint8_t ADC_PIN = 35; // GPIO35 — battery divider input
  static const float DIVIDER_MUL = 2.00f;
  static const float VOLTAGE_ALPHA = 0.2f;
  static const float PCT_ALPHA = 0.3f;

  static float _voltage = 0.0f;
  static int   _percentage = 0;
  static float filteredVoltage = 0.0f;
  static float filteredPct = 0.0f;
  static bool  seeded = false;

  static bool  extPowerHint = false;
  static bool  extPowerDetected = false;
  static float lastGoodVoltage = 0.0f;
  static int   lastGoodPct = 0;

  // Learned calibration, persisted in NVS
  static float vMin = 4.2f;
  static float vMax = 0.0f;
  static Preferences prefs;

  void loadCalibration() {
    prefs.begin("battery", true);
    vMin = prefs.getFloat("vMin", 4.2f);
    vMax = prefs.getFloat("vMax", 0.0f);
    prefs.end();
  }

  void saveCalibration() {
    prefs.begin("battery", false);
    prefs.putFloat("vMin", vMin);
    prefs.putFloat("vMax", vMax);
    prefs.end();
  }

  void begin() {
    analogReadResolution(12);
    #ifdef ARDUINO_ARCH_ESP32
      analogSetPinAttenuation(ADC_PIN, ADC_11db);
    #endif
    loadCalibration();
    seeded = false;
    extPowerDetected = false;
  }

  void setExternalPowerHint(bool present) {
    extPowerHint = present;
  }

  bool externalPower() {
    return extPowerDetected || extPowerHint;
  }

  void update() {
    int raw = analogRead(ADC_PIN);
    float v_adc = (raw / 4095.0f) * 3.3f;
    float v_batt = v_adc * DIVIDER_MUL;

    if (!seeded) {
      filteredVoltage = v_batt;
      filteredPct = 0.5f;
      lastGoodVoltage = filteredVoltage;
      lastGoodPct = 50;
      seeded = true;
    }

    filteredVoltage = VOLTAGE_ALPHA * v_batt + (1.0f - VOLTAGE_ALPHA) * filteredVoltage;

    // Self‑calibration: only update when running from battery
    if (!externalPower()) {
      if (filteredVoltage > vMax) { vMax = filteredVoltage; saveCalibration(); }
      if (filteredVoltage < vMin) { vMin = filteredVoltage; saveCalibration(); }
    }

    float pct = (vMax > vMin) ? (filteredVoltage - vMin) / (vMax - vMin) : 0.0f;
    pct = constrain(pct, 0.0f, 1.0f);
    filteredPct = PCT_ALPHA * pct + (1.0f - PCT_ALPHA) * filteredPct;

    // Detect “floaty” USB‑fed readings
    extPowerDetected = extPowerHint || (filteredVoltage - lastGoodVoltage > 0.15f);

    if (!extPowerDetected) {
      _voltage = filteredVoltage;
      _percentage = (int)(filteredPct * 100.0f + 0.5f);
      lastGoodVoltage = _voltage;
      lastGoodPct = _percentage;
    } else {
      _voltage = lastGoodVoltage;
      _percentage = lastGoodPct;
    }
  }

  float getVoltage() { return _voltage; }
  int   getPercentage() { return _percentage; }

  bool isCharging() {
    return false; // wire in charger STAT pin if available
  }
}
