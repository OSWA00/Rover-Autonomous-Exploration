#ifndef MOTOR_H
#define MOTOR_H

struct Motor
{
    int enable_pin;
    int enable_pwm_channel;
};

void init_motor(Motor &motor, int enable_pin, int enable_pwm_channel);

void send_pwm(Motor &motor, float u);

#endif