#include "GPS.h"
#include <Arduino.h>
#include <math.h>

GPS::GPS(int rxPin, int txPin, double lat, double lon, double radius)
    : gpsSerial(rxPin, txPin), safeZoneLat(lat), safeZoneLon(lon), safeZoneRadius(radius) {}

void GPS::begin(int baudRate) {
    gpsSerial.begin(baudRate);
}

bool GPS::updateLocation(double& latitude, double& longitude) {
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isValid()) {
                latitude = gps.location.lat();
                longitude = gps.location.lng();
                return true;
            }
        }
    }
    return false;
}

double GPS::calculateDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371e3; // Radio de la Tierra en metros
    double lat1Rad = radians(lat1);
    double lat2Rad = radians(lat2);
    double deltaLat = radians(lat2 - lat1);
    double deltaLon = radians(lon2 - lon1);

    double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
               cos(lat1Rad) * cos(lat2Rad) *
               sin(deltaLon / 2) * sin(deltaLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

bool GPS::isOutsideSafeZone(double latitude, double longitude) {
    return calculateDistance(latitude, longitude, safeZoneLat, safeZoneLon) > safeZoneRadius;
}

double GPS::getDistanceToSafeZone(double latitude, double longitude) {
    return calculateDistance(latitude, longitude, safeZoneLat, safeZoneLon);
}
