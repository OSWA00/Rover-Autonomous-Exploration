#pragma once

namespace velocity_controller {
struct ControllerParameters {
    double integral_error = 0.0;
    double gain_integral = 0.0;
    double gain_proportional = 0.0;
    double velocity_reference = 0.0;
};

void init_controller(ControllerParameters& controller, double gain_proportional,
                     double gain_integral);

double calculate_u(ControllerParameters& controller, double current_velocity,
                   double desired_velocity, double time_delta);

}// namespace velocity_controller