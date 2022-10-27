#include "Encoder.h"
#include "Arduino.h"

void init_encoder(Encoder &encoder, unsigned int channel_A_pin)
{
    encoder.channel_A_pin = channel_A_pin;
    encoder.pulses = 0x0;
    pinMode(encoder.channel_A_pin, INPUT_PULLUP);
}

float calculate_omega(Encoder &encoder, float time_delta)
{
    unsigned int total_pulses = 0x276; // 630 pulses
    float omega = 2 * M_PI * encoder.pulses / (time_delta * total_pulses);
    encoder.pulses = 0x0;
    return omega;
}
