#include <Arduino.h>
#include "Encoder.h"

#include <ros.h>
#include <std_msgs/String.h>

Encoder ENCODER_LEFT;
Encoder ENCODER_RIGHT;

void encoder_right_isr_handler(void);
void encoder_left_isr_handler(void);

void setup()
{
    init_encoder(ENCODER_RIGHT, 0x0, 0x1);
    init_encoder(ENCODER_LEFT, 0x2, 0x3);

    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT.channel_A_pin), encoder_right_isr_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT.channel_A_pin), encoder_left_isr_handler, RISING);
}

void loop()
{
}

void encoder_right_isr_handler(void)
{
    ENCODER_RIGHT.pulses++;
}

void encoder_left_isr_handler(void)
{
    ENCODER_LEFT.pulses++;
}
