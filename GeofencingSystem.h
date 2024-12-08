#ifndef GEOFENCINGSYSTEM_H
#define GEOFENCINGSYSTEM_H

#include "GPS.h"
#include "LED.h"
#include "Timer.h"

class GeofencingSystem {
private:
    GPS gps;
    LED led;
    Timer messageTimer;

public:
    GeofencingSystem(int gpsRxPin, int gpsTxPin, int ledPin, double safeLat, double safeLon, double safeRadius, unsigned long messageInterval);
    void begin();
    void run();
};

#endif
