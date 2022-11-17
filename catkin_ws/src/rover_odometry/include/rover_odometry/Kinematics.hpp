#pragma once

#include <memory>

namespace rover_odometry {

class Kinematics {
   public:
    Kinematics();

    virtual ~Kinematics();

    void addRobotParameters(const double &wheelSeparation, const double &wheelRadius);

    double estimateWheelLinearVelocity(const double &omega);

    void estimateLinearVelocity();

    void estimateAngularVelocity();

    void estimatePosition(const double &deltaTime);

    void setRightWheelEstVel(const double &velocity);

    void setLeftWheelEstVel(const double &velocity);

    double getXEstPose();

    double getYEstPose();

    double getThetaEstPose();

    double getVelocityEstX();

    double getVelocityEstY();

    double getVelocityEstTheta();

   private:
    struct RobotParameters;

    struct RobotOdometry;

    std::unique_ptr<RobotParameters> robotParamaters_;

    std::unique_ptr<RobotOdometry> robotOdometry_;
};
}  // namespace rover_odometry