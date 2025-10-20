#include "clock.h"
#include "gps.h"


static int year = 2025;
static int month = 1;
static int day = 1;
static int hour = 0;
static int minute = 0;
static int second = 0;

static unsigned long lastMillis = 0;
static char datetimeString[25] = "0000-00-00 00:00:00";

const int daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

bool isLeapYear(int y) {
    return ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
}

void incrementTime() {
    second++;
    if (second >= 60) {
        second = 0;
        minute++;
        if (minute >= 60) {
            minute = 0;
            hour++;
            if (hour >= 24) {
                hour = 0;
                day++;
                int dim = daysInMonth[month - 1];
                if (month == 2 && isLeapYear(year)) dim = 29;
                if (day > dim) {
                    day = 1;
                    month++;
                    if (month > 12) {
                        month = 1;
                        year++;
                    }
                }
            }
        }
    }
}

void clock_begin() {
    lastMillis = millis();
}

void clock_update() {
    unsigned long now = millis();
    if (now - lastMillis >= 1000) {
        lastMillis += 1000;
        update_time_from_gps();  // Sync time with GPS when possible
        incrementTime();

        snprintf(datetimeString, sizeof(datetimeString),
                 "%02d-%02d-%04d %02d:%02d:%02d",
                 day, month, year, hour, minute, second);
    }
}

const char* get_datetime_string() {
    return datetimeString;
}

void clock_set_datetime(int d, int mo, int y,   int h, int mi, int s) {
    
    day = d;
    month = mo;
    year = y;
    hour = h;
    minute = mi;
    second = s;
    lastMillis = millis();
}

// **NEW FUNCTION: Update time using GPS**
void update_time_from_gps() {
    if (gps.isValid()) {
        year = gps.getYear();  // GPS doesn't provide year, keeping system-tracked
        month = gps.getMonth();
        day = gps.getDay();
        hour = gps.getHour();
        minute = gps.getMinute();
        second = gps.getSecond();

        // Apply BST adjustment (March-October)
        if (month >= 3 && month <= 10) {
            hour += 1;
        }
    }
}

