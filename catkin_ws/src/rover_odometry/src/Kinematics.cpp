#include "rover_odometry/Kinematics.hpp"

namespace rover_odometry {
struct Kinematics::RobotParameters {
    float wheelSeparation_;
    float wheelRadius_;
};

struct Kinematics::RobotOdometry {
    float right_wheel_est_vel_;
    float left_wheel_est_vel_;
    float linear_est_vel;
    float angular_est_vel;
};

Kinematics::Kinematics() {
    robotParamaters_ = std::unique_ptr<RobotParameters>();
    robotOdometry_ = std::unique_ptr<RobotOdometry>();
}

Kinematics::~Kinematics() = default;

void Kinematics::addRobotParameters(float wheelSeparation, float wheelRadius) {
    robotParamaters_->wheelSeparation_ = wheelSeparation;
    robotParamaters_->wheelRadius_ = wheelRadius;
}

float Kinematics::estimateWheelLinearVelocity(float w) {
    return w * robotParamaters_->wheelRadius_;
}

void Kinematics::estimateLinearVelocity() {
    robotOdometry_->linear_est_vel = (robotOdometry_->right_wheel_est_vel_ +
                                      robotOdometry_->left_wheel_est_vel_) /
                                     2;
}

void Kinematics::estimateAngularVelocity() {
    robotOdometry_->angular_est_vel = (robotOdometry_->right_wheel_est_vel_ -
                                       robotOdometry_->left_wheel_est_vel_) /
                                      robotParamaters_->wheelSeparation_;
}
}  // namespace rover_odometry