#include "kinematics.h"

namespace kinematics {

double convert_omega_to_vel(double omega)
{
    double wheel_radio = 0.053;
    return omega * wheel_radio;
}

double calculate_frontal_velocity(double vel_right, double vel_left)
{
    return vel_right + vel_left / 2.0;
}

double calculate_frontal_omega(double vel_right, double vel_left)
{
    double robot_length = 0.1505;
    return (vel_left - vel_right) / (2.0 * robot_length);
}

double calculate_right_velocity(double velocity, double omega)
{
    double robot_length = 0.1505;
    return velocity + 0.5 * robot_length * omega;
}

double calculate_left_velocity(double velocity, double omega)
{
    double robot_length = 0.1505;
    return velocity - 0.5 * robot_length * omega;
}
}// namespace kinematics