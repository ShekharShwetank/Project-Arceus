#include "ppm.h"

static InterruptIn ppm_input(PPM_PIN);
static Timer ppm_timer;

static volatile int ppm_channels[MAX_CHANNELS] = {1500};
static volatile uint8_t ppm_channel_index = 0;
static volatile int last_rise_time = 0;

void ppm_isr_rise() {
    int now = ppm_timer.elapsed_time().count();
    int duration = now - last_rise_time;
    last_rise_time = now;

    if (duration > 3000) {
        ppm_channel_index = 0;
    } else {
        if (ppm_channel_index < MAX_CHANNELS) {
            ppm_channels[ppm_channel_index] = duration;
            ppm_channel_index++;
        }
    }
}

void ppm_init() {
    ppm_timer.start();
    ppm_input.rise(&ppm_isr_rise);
}

int ppm_read(uint8_t channel) {
    if (channel >= MAX_CHANNELS) return 1500;
    return ppm_channels[channel];
}
