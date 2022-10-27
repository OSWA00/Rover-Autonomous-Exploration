#include <Arduino.h>
#include "Encoder.h"

// #include <ros.h>
// #include <std_msgs/String.h>

Encoder ENCODER_LEFT;
Encoder ENCODER_RIGHT;

unsigned long int TIME_CURRENT;
float TIME_DELTA;
unsigned long int TIME_LAST;

void encoder_right_isr_handler();
void encoder_left_isr_handler();

void setup()
{
    Serial.begin(115200);
    TIME_LAST = millis();

    init_encoder(ENCODER_RIGHT, 0x0);
    init_encoder(ENCODER_LEFT, 0x2);

    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT.channel_A_pin), encoder_right_isr_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT.channel_A_pin), encoder_left_isr_handler, RISING);
}

void loop()
{
    TIME_CURRENT = millis();
    TIME_DELTA = (TIME_CURRENT - TIME_LAST) * 0.001;
    TIME_LAST = TIME_CURRENT;

    float omega_right = calculate_omega(ENCODER_RIGHT, TIME_DELTA);
    float omega_left = calculate_omega(ENCODER_LEFT, TIME_DELTA);
    Serial.print("Omega right: ");
    Serial.println(omega_right);
    Serial.print("Omega left: ");
    Serial.println(omega_left);

    delay(10);
}

void encoder_right_isr_handler()
{
    ENCODER_RIGHT.pulses++;
}

void encoder_left_isr_handler()
{
    ENCODER_LEFT.pulses++;
}
