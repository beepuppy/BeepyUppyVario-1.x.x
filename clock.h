#ifndef CLOCK_H
#define CLOCK_H

#include <Arduino.h>
#include "gps.h"


extern GPSHandler gps;  // Reference the GPS object from the main .ino file

// Call once in setup
void clock_begin();

// Call every loop or once per second
void clock_update();

// Get formatted "YYYY-MM-DD HH:MM:SS" string
const char* get_datetime_string();

// Set full time manually (e.g. from GPS or NTP)
void clock_set_datetime(int year, int month, int day, int hour, int minute, int second);

// Get time from GPS and adjust for UK timezone (BST/GMT)
void update_time_from_gps();

#endif
