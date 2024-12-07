#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Configuración del GPS
class GPSModule {
public:
    GPSModule(int rxPin, int txPin) : gpsSerial(rxPin, txPin), gps() {
        gpsSerial.begin(9600);
    }

    void update() {
        while (gpsSerial.available() > 0) {
            if (gps.encode(gpsSerial.read())) {
                if (gps.location.isValid()) {
                    latitude = gps.location.lat();
                    longitude = gps.location.lng();
                }
            }
        }
    }

    double getLatitude() const { return latitude; }
    double getLongitude() const { return longitude; }
    bool isValid() const { return gps.location.isValid(); }

private:
    SoftwareSerial gpsSerial;
    TinyGPSPlus gps;
    double latitude = 0.0;
    double longitude = 0.0;
};

// Configuración del LED
class LEDIndicator {
public:
    LEDIndicator(int pin) : ledPin(pin) {
        pinMode(ledPin, OUTPUT);
        digitalWrite(ledPin, LOW);
    }

    void turnOn() { digitalWrite(ledPin, HIGH); }
    void turnOff() { digitalWrite(ledPin, LOW); }

private:
    int ledPin;
};

// Clase para manejar la geofencing
class GeoFence {
public:
    GeoFence(double safeLat, double safeLon, double radius)
        : safeLatitude(safeLat), safeLongitude(safeLon), safeRadius(radius) {}

    bool isOutsideSafeZone(double lat, double lon) const {
        return calculateDistance(lat, lon, safeLatitude, safeLongitude) > safeRadius;
    }

    double getDistance(double lat, double lon) const {
        return calculateDistance(lat, lon, safeLatitude, safeLongitude);
    }

private:
    double safeLatitude;
    double safeLongitude;
    double safeRadius;

    double calculateDistance(double lat1, double lon1, double lat2, double lon2) const {
        const double R = 6371e3; // Radio de la Tierra en metros
        double lat1Rad = toRadians(lat1);
        double lat2Rad = toRadians(lat2);
        double deltaLat = toRadians(lat2 - lat1);
        double deltaLon = toRadians(lon2 - lon1);

        double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
                   cos(lat1Rad) * cos(lat2Rad) *
                   sin(deltaLon / 2) * sin(deltaLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return R * c; // Distancia en metros
    }

    double toRadians(double degrees) const {
        return degrees * (M_PI / 180);
    }
};

// Constantes para la zona segura
const double SAFE_ZONE_LAT = 40.416775; // Coordenada de latitud de la zona segura
const double SAFE_ZONE_LON = -3.703790; // Coordenada de longitud de la zona segura
const double SAFE_ZONE_RADIUS = 500.0; // Radio de la zona segura en metros

// Pines para el GPS y el LED
const int GPS_RX_PIN = 6; // Pin RX del Arduino conectado al TX del GPS
const int GPS_TX_PIN = 7; // Pin TX del Arduino conectado al RX del GPS
const int LED_PIN = 9;    // Pin del LED

// Configuración de las clases
GPSModule gps(GPS_RX_PIN, GPS_TX_PIN);
LEDIndicator led(LED_PIN);
GeoFence geofence(SAFE_ZONE_LAT, SAFE_ZONE_LON, SAFE_ZONE_RADIUS);

void setup() {
    Serial.begin(9600);
    Serial.println("Iniciando sistema...");
}

void loop() {
    gps.update();

    if (gps.isValid()) {
        double lat = gps.getLatitude();
        double lon = gps.getLongitude();
        double distance = geofence.getDistance(lat, lon);

        Serial.print("Latitud: ");
        Serial.println(lat);
        Serial.print("Longitud: ");
        Serial.println(lon);

        Serial.print("Distancia a la zona segura: ");
        Serial.print(distance);
        Serial.println(" metros");

        if (geofence.isOutsideSafeZone(lat, lon)) {
            Serial.println("¡Fuera de la zona segura!");
            led.turnOn();
        } else {
            Serial.println("Dentro de la zona segura.");
            led.turnOff();
        }
    } else {
        Serial.println("Esperando datos GPS...");
    }

    delay(1000); // Pausa entre lecturas
}
