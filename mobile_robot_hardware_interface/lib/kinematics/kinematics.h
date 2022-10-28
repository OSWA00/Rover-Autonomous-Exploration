#ifndef KINEMATICS_H
#define KINEMATICS_H

float convert_omega_to_vel(float omega);

float calculate_frontal_velocity(float vel_right, float vel_left);

float calculate_frontal_omega(float vel_right, float vel_left);

float calculate_right_velocity(float velocity, float omega);

float calculate_left_velocity(float velocity, float omega);

#endif