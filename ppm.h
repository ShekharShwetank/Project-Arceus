#ifndef PPM_H
#define PPM_H

#include "mbed.h"

#define MAX_CHANNELS 8
#define PPM_PIN PA_0

void ppm_init();
int ppm_read(uint8_t channel);

#endif
