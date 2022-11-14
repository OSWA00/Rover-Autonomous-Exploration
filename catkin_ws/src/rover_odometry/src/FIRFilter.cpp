#include "rover_odometry/FIRFilter.hpp"

namespace rover_odometry {
FIRFilter::FIRFilter() : filterValues_(51, 1), filterCoefficients_(1, 51) {
    filterValues_.setZero();
    filterValues_(50, 0) = 1.0;  // Set last value to 1.0

    filterCoefficients_.setConstant(1.0f);
    filterValues_(48, 0) = 4.5f;
}

FIRFilter::~FIRFilter() {}

float FIRFilter::filterWheelAngularVelocity(float omega) {
    float n_rows = filterValues_.rows(); 
    
    for (size_t i = n_rows - 2; i > 0; --i) {
        filterValues_(i, 0) = filterValues_(i - 1, 0);
    }

    filterValues_(0, 0) = omega;



    // Eigen::Matrix fitleredOmega = 

    return 0.0f;
}
};  // namespace rover_odometry
