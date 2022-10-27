
#ifndef ENCODER_H
#define ENCODER_H

struct Encoder
{
    unsigned int channel_A_pin;
    unsigned int pulses;
};

void init_encoder(Encoder &encoder, unsigned int channel_A_pin);

float calculate_omega(Encoder &encoder, float time_delta);

#endif
