#pragma once
namespace kinematics {
double convert_omega_to_vel(double omega);

double calculate_frontal_velocity(double vel_right, double vel_left);

double calculate_frontal_omega(double vel_right, double vel_left);

double calculate_right_velocity(double velocity, double omega);

double calculate_left_velocity(double velocity, double omega);
}  // namespace kinematics