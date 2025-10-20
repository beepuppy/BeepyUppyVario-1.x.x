#include "Beep.h"
#include <driver/ledc.h>
#include <Arduino.h>
#include <lvgl.h>

// Local channel state (single Beep instance assumed)
static uint32_t s_lastFreq = 0;
static bool     s_toneOn   = false;

// Non-blocking beep scheduler
static unsigned long s_beepOnUntil  = 0;
static unsigned long s_nextBeepAt   = 0;

Beep::Beep(uint8_t pin, uint8_t channel)
  : _pin(pin),
    _channel(channel),
    _lastBeepTime(0),
    _isAlarmActive(false),
    vsIndex(0)
{
  // 1 kHz base, 8-bit resolution (stronger output)
  ledcSetup(_channel, 1000, 8);
  ledcAttachPin(_pin, _channel);

  s_lastFreq = 1000;
  s_toneOn   = false;

  for (int i = 0; i < 5; i++) vsBuffer[i] = 0.0f;
  settingsPanel = nullptr;
  beepVolume = 200;  // slightly high default for good audibility

  s_beepOnUntil = 0;
  s_nextBeepAt  = 0;
}

void Beep::stop() {
  ledcWrite(_channel, 0);
  _isAlarmActive = false;
  s_toneOn = false;
}

void Beep::playTone(uint32_t freq) {
  if (freq != s_lastFreq) {
    // Re-setup the timer fresh each tone for best amplitude
    ledcDetachPin(_pin);
    ledcSetup(_channel, freq, 8);    // 8-bit resolution at this exact tone freq
    ledcAttachPin(_pin, _channel);
    s_lastFreq = freq;
  }

  // Map volume (0–255) to 40–100% duty
  const uint8_t dutyPct = map(beepVolume, 0, 255, 40, 100);
  const uint8_t duty = (255 * dutyPct) / 100;
  ledcWrite(_channel, duty);

  s_toneOn = true;
}


float Beep::mapClimbToFreq(float vs) {
  vs = constrain(vs, 0.1f, 5.0f);
  return 2000.0f + ((vs - 0.1f) / (5.0f - 0.1f)) * (2800.0f - 2000.0f);
}


void Beep::update(float vs) {
  unsigned long now = millis();

  // Rolling average over 5 samples
  vsBuffer[vsIndex] = vs;
  vsIndex = (uint8_t)((vsIndex + 1) % 5);

  float vsAvg = 0;
  for (int i = 0; i < 5; i++) vsAvg += vsBuffer[i];
  vsAvg /= 2.0f;

  // Descent alarm
  if (vsAvg < -1.0f) {
    if (!_isAlarmActive || s_lastFreq != 300) {
      playTone(300);
      _isAlarmActive = true;
    }
    return;
  } else {
    _isAlarmActive = false;
  }

  // Silence near level
  if (vsAvg < 0.1f) {
    stop();
    return;
  }

  // Non-blocking beep cadence
  const uint16_t minOnMs  = 40;    // slightly longer for stronger note
  const uint16_t maxOnMs  = 220;
  const uint16_t onMs     = (uint16_t)map(beepVolume, 0, 255, minOnMs, maxOnMs);
  const uint16_t periodMs = 500;

  // Turn off if time elapsed
  if (s_toneOn && now >= s_beepOnUntil) {
    stop();
  }

  // Start a new beep if time and off
  if (!s_toneOn && now >= s_nextBeepAt) {
    float freq = mapClimbToFreq(vsAvg);
    playTone((uint32_t)freq);
    s_beepOnUntil = now + onMs;
    s_nextBeepAt  = now + periodMs;
  }
}

void Beep::setVolume(uint8_t val) {
  beepVolume = constrain(val, 0, 255);
}

uint8_t Beep::getVolume() {
  return beepVolume;
}

void Beep::closeSettingsUI() {
  if (settingsPanel) {
    lv_obj_add_flag(settingsPanel, LV_OBJ_FLAG_HIDDEN);
    stop();
  }
}

void Beep::showSettingsUI(lv_obj_t* parent) {
  if (settingsPanel) {
    lv_obj_clear_flag(settingsPanel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(settingsPanel);
    return;
  }

  settingsPanel = lv_obj_create(parent);
  lv_obj_set_size(settingsPanel, 300, 150);
  lv_obj_center(settingsPanel);
  lv_obj_set_style_bg_color(settingsPanel, lv_color_hex(0xF0F0F0), 0);
  lv_obj_set_scrollbar_mode(settingsPanel, LV_SCROLLBAR_MODE_OFF);

  // Title
  lv_obj_t* label = lv_label_create(settingsPanel);
  lv_label_set_text(label, "Beep Volume");
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 10);

  // Volume slider
  lv_obj_t* slider = lv_slider_create(settingsPanel);
  lv_obj_set_width(slider, 200);
  lv_obj_align(slider, LV_ALIGN_CENTER, 0, 10);
  lv_slider_set_range(slider, 0, 255);
  lv_slider_set_value(slider, beepVolume, LV_ANIM_OFF);

  // Immediate apply on change
  lv_obj_add_event_cb(
    slider, [](lv_event_t* e) {
      Beep* self = static_cast<Beep*>(lv_event_get_user_data(e));
      lv_obj_t* s = lv_event_get_target(e);
      uint16_t val = lv_slider_get_value(s);
      self->setVolume(val);

      // Light serial feedback
      static unsigned long lastPrint = 0;
      unsigned long now = millis();
      if (now - lastPrint > 120) {
        Serial.printf("[Beep] Volume -> %u\n", val);
        lastPrint = now;
      }
    },
    LV_EVENT_VALUE_CHANGED, this);

  // Released: confirm final value
  lv_obj_add_event_cb(
    slider, [](lv_event_t* e) {
      Beep* self = static_cast<Beep*>(lv_event_get_user_data(e));
      lv_obj_t* s = lv_event_get_target(e);
      uint16_t val = lv_slider_get_value(s);
      self->setVolume(val);
      Serial.printf("[Beep] Volume set to %u (final)\n", val);
    },
    LV_EVENT_RELEASED, this);

  // Close button
  lv_obj_t* closeBtn = lv_btn_create(settingsPanel);
  lv_obj_set_size(closeBtn, 60, 30);
  lv_obj_align(closeBtn, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_t* closeLabel = lv_label_create(closeBtn);
  lv_label_set_text(closeLabel, "Close");
  lv_obj_center(closeLabel);

  lv_obj_add_event_cb(
    closeBtn, [](lv_event_t* e) {
      Beep* self = static_cast<Beep*>(lv_event_get_user_data(e));
      self->closeSettingsUI();
      Serial.println("[Beep] Settings panel closed");
    },
    LV_EVENT_CLICKED, this);
}
