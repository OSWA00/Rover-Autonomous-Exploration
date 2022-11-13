#include "rover_odometry/FIRFilter.hpp"

namespace rover_odometry {
    FIRFilter::FIRFilter() : filterValues_(51, 1), filterCoefficients_(1, 51) {

    }

    FIRFilter::~FIRFilter() {}

    float FIRFilter::filterWheelAngularVelocity(float omega) {
        return float(0.0);
    }
};
