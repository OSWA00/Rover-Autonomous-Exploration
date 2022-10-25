
#ifndef ENCODER_H
#define ENCODER_H

#include "stdint.h"

struct Encoder
{
    uint8_t channel_A_pin;
    uint8_t channel_B_pin;
    uint32_t pulses;
};

void init_encoder(Encoder &encoder, uint8_t channel_A_pin, uint8_t channel_B_pin);

#endif
