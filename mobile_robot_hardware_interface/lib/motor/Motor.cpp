#include "Motor.h"
namespace motor
{
    void init_motor(Motor &motor, int enable_pin, int enable_pwm_channel)
    {
        int pwm_frequency = 0x3E8;
        int pwm_resolution = 0x8;

        motor.enable_pin = enable_pin;
        motor.enable_pwm_channel = enable_pwm_channel;

        pinMode(motor.enable_pin, OUTPUT);

        ledcSetup(motor.enable_pwm_channel, pwm_frequency, pwm_resolution);
        ledcAttachPin(motor.enable_pin, motor.enable_pwm_channel);

        digitalWrite(motor.enable_pin, LOW);
    }

    void send_pwm(Motor &motor, double u)
    {
        int duty_cycle = -u * 255;
        ledcWrite(motor.enable_pwm_channel, duty_cycle);
    }
}
