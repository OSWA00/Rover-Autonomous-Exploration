#include "vel_controller.h"

void init_controller(Vel_controller controller, float gain_proportional, float gain_integral)
{
    controller.gain_proportional = gain_proportional;
    controller.gain_integral = gain_integral;
}

float calculate_u(Vel_controller &controller, float current_vel, float desired_vel, float time_delta)
{
    float error = desired_vel - current_vel;
    controller.integral_error += error * time_delta;

    float u_proportional = controller.gain_proportional * error;
    float u_integral = controller.gain_integral * controller.integral_error;
    float u = u_proportional + u_integral;

    if (u > 1)
    {
        u = 1.0;
    }

    if (u < 0)
    {
        u = 0;
    }

    return u;
}