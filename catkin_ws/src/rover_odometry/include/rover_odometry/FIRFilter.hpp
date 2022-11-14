#pragma once

#include <Eigen/Core>

namespace rover_odometry {
    class FIRFilter
    {
    private:
        Eigen::MatrixXf filterValues_;
        Eigen::MatrixXf filterCoefficients_;
    public:
        FIRFilter();
        virtual ~FIRFilter();
        float filterWheelAngularVelocity(float omega);
    };
}