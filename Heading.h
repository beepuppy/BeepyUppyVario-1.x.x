#ifndef HEADING_H
#define HEADING_H

#include <Arduino.h>

// Call once in setup
void initHeadingSensors();

// Call every loop
void updateHeading();

void setHeadingOffset(float offsetDeg);


// Filtered heading in 0..360 degrees
float getHeadingDegrees();

// Continuous heading (... -360, 0, 360, ...) for dial
float getHeadingUnwrappedDegrees();


#endif
