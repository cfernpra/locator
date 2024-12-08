#ifndef GPS_H
#define GPS_H

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

class GPS {
private:
    TinyGPSPlus gps;
    SoftwareSerial gpsSerial;
    const double safeZoneLat;
    const double safeZoneLon;
    const double safeZoneRadius;

    double calculateDistance(double lat1, double lon1, double lat2, double lon2);

public:
    GPS(int rxPin, int txPin, double lat, double lon, double radius);
    void begin(int baudRate = 9600);
    bool updateLocation(double& latitude, double& longitude);
    bool isOutsideSafeZone(double latitude, double longitude);
    double getDistanceToSafeZone(double latitude, double longitude);
};

#endif
