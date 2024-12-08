#include "GeofencingSystem.h"
#include <Arduino.h>

GeofencingSystem::GeofencingSystem(int gpsRxPin, int gpsTxPin, int ledPin, double safeLat, double safeLon, double safeRadius, unsigned long messageInterval)
    : gps(gpsRxPin, gpsTxPin, safeLat, safeLon, safeRadius),
      led(ledPin),
      messageTimer(messageInterval) {}

void GeofencingSystem::begin() {
    Serial.begin(9600);
    gps.begin();
    led.begin();
    Serial.println("Iniciando sistema...");
}

void GeofencingSystem::run() {
    double latitude, longitude;

    if (gps.updateLocation(latitude, longitude)) {
        double distance = gps.getDistanceToSafeZone(latitude, longitude);

        if (gps.isOutsideSafeZone(latitude, longitude)) {
            if (messageTimer.isReady()) {
                Serial.println("¡Fuera de la zona segura!");
                Serial.print("Distancia a la zona segura: ");
                Serial.print(distance);
                Serial.println(" metros");
            }
            led.on();
        } else {
            if (messageTimer.isReady()) {
                Serial.println("Dentro de la zona segura.");
            }
            led.off();
        }
    } else {
        if (messageTimer.isReady()) {
            //Serial.println("Esperando datos GPS...");
        }
    }
}
