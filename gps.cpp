#include "gps.h"
#include <string.h>
#include <stdlib.h> // for atof

GPSHandler::GPSHandler(int rxPin, int txPin, long baudRate) {
    gpsSerial = &Serial2;
    gpsSerial->begin(baudRate, SERIAL_8N1, rxPin, txPin);
}

void GPSHandler::update() {
    static char gsaBuf[80];
    static uint8_t gsaPos = 0;

    while (gpsSerial->available()) {
        char c = gpsSerial->read();
        gps.encode(c);

        // --- VDOP NMEA parser tap ---
        if (c == '$') { gsaPos = 0; } // start of sentence
        if (gsaPos < sizeof(gsaBuf) - 1) {
            gsaBuf[gsaPos++] = c;
            gsaBuf[gsaPos] = '\0';
            if (c == '\n' && strstr(gsaBuf, "GSA")) {
                parseGSA(gsaBuf);
            }
        }
    }
}

void GPSHandler::parseGSA(const char* sentence) {
    // GSA: $GPGSA,A,3,xx,xx,...,PDOP,HDOP,VDOP
    // We'll count commas to reach field 17 (VDOP)
    uint8_t commas = 0;
    const char* p = sentence;
    while (*p && commas < 17) {
        if (*p == ',') commas++;
        p++;
    }
    if (commas == 17) {
        latestVDOP = atof(p);
    }
}

bool GPSHandler::isValid() {
    return gps.location.isValid();
}

double GPSHandler::getLatitude() {
    return gps.location.lat();
}

double GPSHandler::getLongitude() {
    return gps.location.lng();
}

int GPSHandler::getSatellites() {
    return gps.satellites.value();
}

double GPSHandler::getHDOP() {
    // gps.hdop.value() returns scaled-by-100 in your current setup
    return gps.hdop.value() / 100.0;
}

double GPSHandler::getVDOP() {
    return latestVDOP; // NAN if never parsed yet
}

String GPSHandler::getTimeUTC() {
    if (gps.time.isValid()) {
        char buffer[10];
        snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d",
                 gps.time.hour(), gps.time.minute(), gps.time.second());
        return String(buffer);
    }
    return "No Fix";
}

int GPSHandler::getYear() {
    return 2025;  // GPS doesn't provide the year; fixed assumption
}

int GPSHandler::getMonth() {
    return gps.date.isValid() ? gps.date.month() : 0;
}

int GPSHandler::getDay() {
    return gps.date.isValid() ? gps.date.day() : 0;
}

int GPSHandler::getHour() {
    return gps.time.isValid() ? gps.time.hour() : 0;
}

int GPSHandler::getMinute() {
    return gps.time.isValid() ? gps.time.minute() : 0;
}

int GPSHandler::getSecond() {
    return gps.time.isValid() ? gps.time.second() : 0;
}

double GPSHandler::getAltitude() {
    return gps.altitude.isValid() ? gps.altitude.meters() : -1;  // -1 if invalid
}

double GPSHandler::getSpeed() {
    return gps.speed.isValid() ? gps.speed.kmph() : 0.0;
}

double GPSHandler::getCourse() {
    return gps.course.isValid() ? gps.course.deg() : -1.0;
}
