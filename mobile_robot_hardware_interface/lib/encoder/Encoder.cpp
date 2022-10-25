#include "Encoder.h"
#include "Arduino.h"

void init_encoder(Encoder &encoder, uint8_t channel_A_pin, uint8_t channel_B_pin)
{
    encoder.channel_A_pin = channel_A_pin;
    encoder.channel_B_pin = channel_B_pin;
    encoder.pulses = 0x0;
    pinMode(encoder.channel_A_pin, INPUT_PULLUP);
    pinMode(encoder.channel_B_pin, INPUT);
}

// float_t encoders::calculate_omega(Encoder &encoder, uint32_t delta_time)
// {
//     uint16_t total_pulses = 0x276;
//     float_t omega = 2 * M_PI * encoder.pulses / (delta_time * total_pulses);
//     encoder.pulses = 0;
//     return omega;
// }
