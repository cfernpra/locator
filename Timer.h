#ifndef TIMER_H
#define TIMER_H

class Timer {
private:
    unsigned long lastTime;
    unsigned long interval;

public:
    Timer(unsigned long ms);
    bool isReady();
};

#endif
