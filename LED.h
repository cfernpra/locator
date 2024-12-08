#ifndef LED_H
#define LED_H

class LED {
private:
    int pin;

public:
    LED(int ledPin);
    void begin();
    void on();
    void off();
};

#endif
