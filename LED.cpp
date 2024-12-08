#include "LED.h"
#include <Arduino.h>

LED::LED(int ledPin) : pin(ledPin) {}

void LED::begin() {
    pinMode(pin, OUTPUT);
    off();
}

void LED::on() {
    digitalWrite(pin, HIGH);
}

void LED::off() {
    digitalWrite(pin, LOW);
}
