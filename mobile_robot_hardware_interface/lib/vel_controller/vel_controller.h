#ifndef VEL_CONTROLLER_H
#define VEL_CONTROLLER_H

struct Vel_controller
{
    float integral_error;
    float gain_integral;
    float gain_proportional;
};

void init_controller(Vel_controller &controller, float gain_proportional, float gain_integral);

float calculate_u(Vel_controller &controller, float current_vel, float desired_vel, float time_delta);

#endif