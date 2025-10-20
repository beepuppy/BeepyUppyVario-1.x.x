#pragma once
#include <Arduino.h>

namespace battery {
  void begin();            // no pin arg — fixed to GPIO35
  void update();

  float getVoltage();      // filtered voltage (V)
  int   getPercentage();   // 0–100 %

  bool  isCharging();      // optional: wire in a pin if available
  bool  externalPower();   // true if USB or “floaty” reading detected

  void  setExternalPowerHint(bool present); // optional VBUS sense
}
