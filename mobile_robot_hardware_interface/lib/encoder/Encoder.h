#pragma once
#include "Arduino.h"

namespace encoder {
struct Encoder {
    unsigned int channel_A_pin;
    unsigned int pulses = 0x0;
};

void init_encoder(Encoder& encoder, unsigned int channel_A_pin);

double calculate_omega(Encoder& encoder, double time_delta);
}// namespace encoder
