#include "Timer.h"
#include <Arduino.h>

Timer::Timer(unsigned long ms) : interval(ms), lastTime(0) {}

bool Timer::isReady() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= interval) {
        lastTime = currentTime;
        return true;
    }
    return false;
}
