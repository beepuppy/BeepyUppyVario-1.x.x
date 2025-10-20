#pragma once
#include <Arduino.h>
#include <lvgl.h>

class Beep {
public:
    Beep(uint8_t pin, uint8_t channel = 0);
    void update(float vs);
    void stop();

    void setVolume(uint8_t duty);         // 0–255
    uint8_t getVolume();                  // current volume

    void showSettingsUI(lv_obj_t* parent);
    void closeSettingsUI();               // hide (or destroy) the settings panel

private:
    void playTone(uint32_t freq);  // was float
    float mapClimbToFreq(float vs);

    uint8_t _pin;
    uint8_t _channel;
    unsigned long _lastBeepTime;
    bool _isAlarmActive;

    float vsBuffer[2];
    int vsIndex;

    uint8_t beepVolume = 128;             // volume 0–255
    lv_obj_t* settingsPanel = nullptr;    // created once, then shown/hidden

    
};
