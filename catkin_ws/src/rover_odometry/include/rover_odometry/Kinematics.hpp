#pragma once

#include <memory>
#include <vector>

namespace rover_odometry {

class Kinematics {
   public:
    Kinematics();

    virtual ~Kinematics();

    void addRobotParameters(float &wheelSeparation, float &wheelRadius);

    float estimateWheelLinearVelocity(float w);

    void estimateLinearVelocity();

    void estimateAngularVelocity();

    void estimatePosition(float deltaTime);

    void setRightWheelEstVel(float velocity);

    void setLeftWheelEstVel(float velocity);

    float getXEstPose();

    float getYEstPose();

    float getThetaEstPose();

    float getVelocityEstX();

    float getVelocityEstY();

    float getVelocityEstTheta();

    float filterWheelVelocity(float w);

   private:
    struct RobotParameters;

    struct RobotOdometry;

    std::unique_ptr<RobotParameters> robotParamaters_;

    std::unique_ptr<RobotOdometry> robotOdometry_;

    std::vector<float>* wheelLeftFilterValues_;
    std::vector<float>* wheelRightFilterValues_;
};
}  // namespace rover_odometry