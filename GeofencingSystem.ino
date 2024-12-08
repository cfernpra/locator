#include "GeofencingSystem.h"

// Configuración global del sistema
GeofencingSystem geofenceSystem(6, 7, 9, 40.416775, -3.703790, 500.0, 1000);

void setup() {
    geofenceSystem.begin();
}

void loop() {
    geofenceSystem.run();
}
