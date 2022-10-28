#include "kinematics.h"

float convert_omega_to_vel(float omega)
{
    float wheel_radio = 0.04;
    return omega * wheel_radio;
}

float calculate_frontal_velocity(float vel_right, float vel_left)
{
    return vel_right + vel_left / 2.0;
}

float calculate_frontal_omega(float vel_right, float vel_left)
{
    float robot_length = 0.1505;
    return (vel_left - vel_right) / (2.0 * robot_length);
}

float calculate_right_velocity(float velocity, float omega)
{
    float robot_length = 0.1505;
    return velocity + 0.5 * robot_length * omega;
}

float calculate_left_velocity(float velocity, float omega)
{
    float robot_length = 0.1505;
    return velocity - 0.5 * robot_length * omega;
}
