#pragma once

#include <memory>

namespace rover_odometry {

class Kinematics {
   public:
    Kinematics();

    virtual ~Kinematics();

    void addRobotParameters(float length, float wheelRadius);

    float estimateWheelLinearVelocity(float w);

    void estimateLinearVelocity();
    void estimateAngularVelocity();

   private:
    struct RobotParameters;

    struct RobotOdometry;

    std::unique_ptr<RobotParameters> robotParamaters_;

    std::unique_ptr<RobotOdometry> robotOdometry_;
};
}  // namespace rover_odometry