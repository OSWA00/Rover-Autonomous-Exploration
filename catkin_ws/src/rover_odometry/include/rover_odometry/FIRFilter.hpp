#pragma once

#include <Eigen/Core>

namespace rover_odometry {
    class FIRFilter
    {
    private:
        Eigen::VectorXf filterCoefficients_;
        Eigen::VectorXf filterValues_;
    public:
        FIRFilter();
        virtual ~FIRFilter();
        float filterWheelAngularVelocity(float omega);
    };
}