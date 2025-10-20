#pragma once
#include <TinyGPS++.h>
#include <HardwareSerial.h>

class GPSHandler {
public:
    GPSHandler(int rxPin, int txPin, long baudRate);
    void update();

    bool isValid();

    double getLatitude();
    double getLongitude();
    int    getSatellites();
    double getHDOP();   // now returns actual value (e.g. 1.50 instead of 150.00 raw)
    double getVDOP();   // new: actual VDOP value (or NAN if not seen yet)

    String getTimeUTC();
    int getYear();
    int getMonth();
    int getDay();
    int getHour();
    int getMinute();
    int getSecond();

    double getAltitude();
    double getSpeed();
    double getCourse();

private:
    TinyGPSPlus gps;
    HardwareSerial *gpsSerial;
    void parseGSA(const char* sentence);

    double latestVDOP = NAN; // store last VDOP parsed
};

extern GPSHandler gps; 
