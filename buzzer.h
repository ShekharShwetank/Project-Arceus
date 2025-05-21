#ifndef BUZZER_H
#define BUZZER_H

#include "mbed.h"

class Buzzer {
public:
    Buzzer(PinName pin);
    void play_disarm_tune();
private:
    PwmOut buzzer;
    void play_note(float frequency, int duration_ms);
};

#endif