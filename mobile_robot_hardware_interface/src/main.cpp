#include <Arduino.h>
#include "Encoder.h"

#include <ros.h>
#include <std_msgs/String.h>

Encoder ENCODER_LEFT;
Encoder ENCODER_RIGHT;

uint32_t TIME_CURRENT;
uint32_t TIME_DELTA;
uint32_t TIME_LAST;

void encoder_right_isr_handler(void);
void encoder_left_isr_handler(void);

void setup()
{
    TIME_LAST = millis();

    init_encoder(ENCODER_RIGHT, 0x0, 0x1);
    init_encoder(ENCODER_LEFT, 0x2, 0x3);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT.channel_A_pin), encoder_right_isr_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT.channel_A_pin), encoder_left_isr_handler, RISING);
}

void loop()
{
    TIME_CURRENT = millis();
    TIME_DELTA = TIME_CURRENT - TIME_LAST;
    TIME_LAST = TIME_CURRENT;

    float omega_right = calculate_omega(ENCODER_RIGHT, TIME_DELTA);
    float omega_left = calculate_omega(ENCODER_LEFT, TIME_DELTA);
}

void encoder_right_isr_handler(void)
{
    ENCODER_RIGHT.pulses++;
}

void encoder_left_isr_handler(void)
{
    ENCODER_LEFT.pulses++;
}
