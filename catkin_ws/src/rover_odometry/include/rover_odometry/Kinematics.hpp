#pragma once

#include <memory>

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

    float get_x_est_pose();

    float get_y_est_pose();

    float get_theta_est_pose();

    float get_velocity_est_x();

    float get_velocity_est_y();

    float get_velocity_est_theta();

   private:
    struct RobotParameters;

    struct RobotOdometry;

    std::unique_ptr<RobotParameters> robotParamaters_;

    std::unique_ptr<RobotOdometry> robotOdometry_;
};
}  // namespace rover_odometry