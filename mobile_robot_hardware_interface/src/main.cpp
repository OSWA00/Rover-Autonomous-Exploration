#include <Arduino.h>
#include "Encoder.h"
#include "kinematics.h"

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

    init_encoder(ENCODER_RIGHT, 0xC);
    init_encoder(ENCODER_LEFT, 0x2);

    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT.channel_A_pin), encoder_right_isr_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT.channel_A_pin), encoder_left_isr_handler, RISING);
}

void loop()
{
    TIME_CURRENT = millis();
    TIME_DELTA = (TIME_CURRENT - TIME_LAST) * 0.001;
    TIME_LAST = TIME_CURRENT;

    float omega_left = calculate_omega(ENCODER_LEFT, TIME_DELTA);
    float vel_left = convert_omega_to_vel(omega_left);

    float omega_right = calculate_omega(ENCODER_RIGHT, TIME_DELTA);
    float vel_right = convert_omega_to_vel(omega_right);

    Serial.print("Velocity left: ");
    Serial.println(vel_left);

    Serial.print("Velocity right: ");
    Serial.println(vel_right);

    delay(1000);
}

void encoder_right_isr_handler()
{
    ENCODER_RIGHT.pulses++;
}

void encoder_left_isr_handler()
{
    ENCODER_LEFT.pulses++;
}
