#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <math.h>

unsigned long lastMessageTime = 0; // Tiempo de la última impresión
const unsigned long messageInterval = 100; // Intervalo entre mensajes en milisegundos

// Clase para el manejo del GPS

class GPS {
private:
    TinyGPSPlus gps;
    SoftwareSerial gpsSerial;
    const double safeZoneLat;
    const double safeZoneLon;
    const double safeZoneRadius;

public:
    GPS(int rxPin, int txPin, double lat, double lon, double radius)
        : gpsSerial(rxPin, txPin),
          safeZoneLat(lat),
          safeZoneLon(lon),
          safeZoneRadius(radius) {}

    void begin(int baudRate = 9600) {
        gpsSerial.begin(baudRate);
    }

    bool updateLocation(double& latitude, double& longitude) {
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

    double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371e3; // Radio de la Tierra en metros
        double lat1Rad = radians(lat1);
        double lat2Rad = radians(lat2);
        double deltaLat = radians(lat2 - lat1);
        double deltaLon = radians(lon2 - lon1);

        double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
                   cos(lat1Rad) * cos(lat2Rad) *
                   sin(deltaLon / 2) * sin(deltaLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return R * c; // Distancia en metros
    }

    bool isOutsideSafeZone(double latitude, double longitude) {
        double distance = calculateDistance(latitude, longitude, safeZoneLat, safeZoneLon);
        if(lastMessageTime == messageInterval){
        Serial.print("Distancia a la zona segura: ");
        Serial.print(distance);
        Serial.println(" metros");
        lastMessageTime = 0;
        }else {
        lastMessageTime++;
        }
        return distance > safeZoneRadius;

    }
};

// Clase para manejar el LED
class LED {
private:
    int pin;

public:
    LED(int ledPin) : pin(ledPin) {}

    void begin() {
        pinMode(pin, OUTPUT);
        off();
    }

    void on() {
        digitalWrite(pin, HIGH);
    }

    void off() {
        digitalWrite(pin, LOW);
    }
};

// Clase principal para el sistema
class GeofencingSystem {
private:
    GPS gps;
    LED led;

public:
    GeofencingSystem(int gpsRxPin, int gpsTxPin, int ledPin, double safeLat, double safeLon, double safeRadius)
        : gps(gpsRxPin, gpsTxPin, safeLat, safeLon, safeRadius), led(ledPin) {}

    void begin() {
        Serial.begin(9600);
        gps.begin();
        led.begin();
        Serial.println("Iniciando sistema...");
    }

    void run() {
        double latitude, longitude;
        if (gps.updateLocation(latitude, longitude)) {
            if (gps.isOutsideSafeZone(latitude, longitude)) {
        if(lastMessageTime == messageInterval){
                Serial.println("¡Fuera de la zona segura!");
                lastMessageTime = 0;
                }else {
                lastMessageTime++;
                }
                led.on();
            } else {
                Serial.println("Dentro de la zona segura.");
                led.off();
            }
        } else {
            //Serial.println("Esperando datos GPS...");
        }
    }
};

// Configuración global del sistema
GeofencingSystem geofenceSystem(6, 7, 9, 40.416775, -3.703790, 500.0);

void setup() {
    geofenceSystem.begin();
}

void loop() {
    geofenceSystem.run();
}
