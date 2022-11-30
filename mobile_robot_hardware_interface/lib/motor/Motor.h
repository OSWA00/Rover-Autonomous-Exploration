#pragma once
#include "Arduino.h"

namespace motor {
struct Motor {
    int enable_pin;
    int enable_pwm_channel;
};

void init_motor(Motor& motor, int enable_pin, int enable_pwm_channel);

void send_pwm(Motor& motor, double u);
}// namespace motor