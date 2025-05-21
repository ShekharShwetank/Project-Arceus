#include "buzzer.h"
#include "mbed.h"

using namespace std::chrono;

Buzzer::Buzzer(PinName pin) : buzzer(pin) {
    buzzer.write(0.0f);
}

void Buzzer::play_note(float frequency, int duration_ms) {
    if (frequency > 0.0f) {
        buzzer.period(1.0f / frequency);
        buzzer.write(0.5f);
    } else {
        buzzer.write(0.0f);
    }
    ThisThread::sleep_for(milliseconds(duration_ms));
    buzzer.write(0.0f);
}

void Buzzer::play_disarm_tune() {
    float melody[] = { 392, 440, 392, 349, 330 };
    int durations[] = { 200, 200, 200, 200, 400 };

    for (int i = 0; i < 5; i++) {
        play_note(melody[i], durations[i]);
        ThisThread::sleep_for(50ms);
    }
}
