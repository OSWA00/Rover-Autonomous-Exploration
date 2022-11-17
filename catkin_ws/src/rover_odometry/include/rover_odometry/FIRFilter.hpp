#pragma once

#include <Eigen/Core>

namespace rover_odometry {
class FIRFilter {
   private:
    Eigen::VectorXd filterCoefficients_;
    Eigen::VectorXd filterValues_;

   public:
    FIRFilter();
    virtual ~FIRFilter();
    double filterWheelAngularVelocity(const double &omega);
};
}  // namespace rover_odometry