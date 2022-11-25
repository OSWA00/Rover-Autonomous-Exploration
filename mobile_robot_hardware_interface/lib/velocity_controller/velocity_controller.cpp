#include "velocity_controller.h"

namespace velocity_controller {
void init_controller(ControllerParameters &controller, double gain_proportional,
                     double gain_integral) {
    controller.gain_proportional = gain_proportional;
    controller.gain_integral = gain_integral;
}

double calculate_u(ControllerParameters &controller, double current_velocity,
                   double desired_velocity, double time_delta) {
    double error = current_velocity - desired_velocity;

    controller.integral_error += error * time_delta;

    double u_proportional = controller.gain_proportional * error;
    double u_integral = controller.gain_integral * controller.integral_error;
    double u = u_proportional + u_integral;

    if (u < -1) {
        u = -1.0;
    }

    if (u > 0) {
        u = 0;
    }

    return u;
}

} // namespace velocity_controller