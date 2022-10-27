#include <Arduino.h>
#include "Encoder.h"
#include "kinematics.h"
#include "Motor.h"
#include "vel_controller.h"

// #include <ros.h>
// #include <std_msgs/String.h>

Encoder ENCODER_LEFT;
Encoder ENCODER_RIGHT;
Motor MOTOR_RIGHT;
Motor MOTOR_LEFT;
Vel_controller CONTROLLER_MOTOR_RIGHT;
Vel_controller CONTROLLER_MOTOR_LEFT;

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

    init_motor(MOTOR_RIGHT, 0x19, 0x0);
    init_motor(MOTOR_LEFT, 0x5, 0x1);

    init_controller(CONTROLLER_MOTOR_RIGHT, 10, 0.01);
    init_controller(CONTROLLER_MOTOR_LEFT, 10, 0.01);

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

    float u_right = calculate_u(CONTROLLER_MOTOR_RIGHT, vel_right, 0.3, TIME_DELTA);
    float u_left = calculate_u(CONTROLLER_MOTOR_LEFT, vel_left, 0.05, TIME_DELTA);

    send_pwm(MOTOR_RIGHT, u_right);
    send_pwm(MOTOR_LEFT, u_left);

    delay(5);
}

void encoder_right_isr_handler()
{
    ENCODER_RIGHT.pulses++;
}

void encoder_left_isr_handler()
{
    ENCODER_LEFT.pulses++;
}
