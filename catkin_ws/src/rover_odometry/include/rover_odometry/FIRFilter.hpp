#pragma once

#include <Eigen/Core>

namespace rover_odometry {
    class FIRFilter
    {
    private:
        Eigen::Matrix2f filterValues_;
        Eigen::Matrix2f filterCoefficients_;
    public:
        FIRFilter();
        virtual ~FIRFilter();
        float filterWheelAngularVelocity(float omega);
    };
}