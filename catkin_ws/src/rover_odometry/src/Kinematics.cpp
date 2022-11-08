#include "rover_odometry/Kinematics.hpp"

#include <cstdio>

namespace rover_odometry {
struct Kinematics::RobotParameters {
    float wheelSeparation_ = 0.0;
    float wheelRadius_ = 0.0;
};

struct Kinematics::RobotOdometry {
    float right_wheel_est_vel_ = 0.0;
    float left_wheel_est_vel_ = 0.0;
    float linear_est_vel_ = 0.0;
    float angular_est_vel_ = 0.0;
    float x_est_pose_ = 0.0;
    float y_est_pose_ = 0.0;
    float theta_est_pose_ = 0.0;
};

Kinematics::Kinematics() {
    robotParamaters_ = std::unique_ptr<RobotParameters>(new RobotParameters);
    robotOdometry_ = std::unique_ptr<RobotOdometry>(new RobotOdometry);
}

Kinematics::~Kinematics() = default;

void Kinematics::addRobotParameters(float &wheelSeparation, float &wheelRadius) {
    robotParamaters_->wheelRadius_ = wheelRadius;
    robotParamaters_->wheelSeparation_ = wheelSeparation;
}

float Kinematics::estimateWheelLinearVelocity(float w) {
    float linear_velocity = w * robotParamaters_->wheelRadius_;
    return linear_velocity;
}

void Kinematics::estimateLinearVelocity() {
    robotOdometry_->linear_est_vel_ = float((robotOdometry_->right_wheel_est_vel_ +
                                             robotOdometry_->left_wheel_est_vel_) /
                                            2);
}

void Kinematics::estimateAngularVelocity() {
    robotOdometry_->angular_est_vel_ = float((robotOdometry_->right_wheel_est_vel_ -
                                              robotOdometry_->left_wheel_est_vel_) /
                                             robotParamaters_->wheelSeparation_);
}

void Kinematics::setLeftWheelEstVel(float velocity) {
    robotOdometry_->left_wheel_est_vel_ = velocity;
}

void Kinematics::setRightWheelEstVel(float velocity) {
    robotOdometry_->right_wheel_est_vel_ = velocity;
}

}  // namespace rover_odometry
