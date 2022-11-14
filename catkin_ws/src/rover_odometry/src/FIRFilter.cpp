#include "rover_odometry/FIRFilter.hpp"

#include <iostream>

namespace rover_odometry {
FIRFilter::FIRFilter() : filterCoefficients_(51), filterValues_(51) {
    filterValues_.setZero();
    filterValues_(50) = 1.0;  

    // TODO add filter coefficients
    filterCoefficients_.setOnes();  //! REMOVE
    filterCoefficients_ = filterCoefficients_.transpose();
}

FIRFilter::~FIRFilter() {}

float FIRFilter::filterWheelAngularVelocity(float omega) {
    long int n_rows = filterValues_.rows();

    for (size_t i = n_rows - 2; i > 0; --i) {
        filterValues_(i) = filterValues_(i - 1);
    }

    filterValues_(0) = omega;

    float filteredOmega = filterCoefficients_.dot(filterValues_);

    return filteredOmega;
}
}  // namespace rover_odometry
