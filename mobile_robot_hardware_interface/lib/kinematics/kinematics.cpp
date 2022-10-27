#include "kinematics.h"

float convert_omega_to_vel(float omega)
{
    float wheel_radio = 0.04;
    return omega * wheel_radio;
}