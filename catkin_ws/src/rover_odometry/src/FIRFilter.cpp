#include "rover_odometry/FIRFilter.hpp"

namespace rover_odometry {
FIRFilter::FIRFilter() : filterValues_(51, 1), filterCoefficients_(1, 51) {
    filterValues_.setZero();
    filterValues_(50, 0) = 1.0;  // Set last value to 1.0

    filterCoefficients_.setConstant(1.0f);
}

FIRFilter::~FIRFilter() {}

float FIRFilter::filterWheelAngularVelocity(float omega) {
    for (size_t i = 0; i < 48; ++i) {
        float tmp = filterValues_(i + 1, 0);
        filterValues_(i + 1, 0) = filterValues_(i, 0);
        float tmp = filterValues_(i + 1, 0);
    }
    filterValues_(0, 0) = omega;

    Eigen::filterValues_ * filterCoefficients_;
    return float(0.0);
}
};  // namespace rover_odometry
