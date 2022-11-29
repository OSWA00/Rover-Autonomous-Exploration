#include "rover_odometry/Kinematics.hpp"

#include <math.h>

namespace rover_odometry {
struct Kinematics::RobotParameters {
    double wheelSeparation_ = 0.0;
    double wheelRadius_ = 0.0;
};

struct Kinematics::RobotOdometry {
    double right_wheel_est_vel_ = 0.0;
    double left_wheel_est_vel_ = 0.0;
    double linear_est_vel_ = 0.0;
    double angular_est_vel_ = 0.0;
    double x_est_pose_ = 0.0;
    double y_est_pose_ = 0.0;
    double theta_est_pose_ = 0.0;
    double velocity_est_x = 0.0;
    double velocity_est_y = 0.0;
};

Kinematics::Kinematics()
{
    robotParamaters_ = std::unique_ptr<RobotParameters>(new RobotParameters);
    robotOdometry_ = std::unique_ptr<RobotOdometry>(new RobotOdometry);
}

Kinematics::~Kinematics() = default;

void Kinematics::addRobotParameters(const double& wheelSeparation,
                                    const double& wheelRadius)
{
    robotParamaters_->wheelRadius_ = wheelRadius;
    robotParamaters_->wheelSeparation_ = wheelSeparation;
}

double Kinematics::estimateWheelLinearVelocity(const double& omega)
{
    float linear_velocity = omega * robotParamaters_->wheelRadius_;
    return linear_velocity;
}

void Kinematics::estimateLinearVelocity()
{
    robotOdometry_->linear_est_vel_ =
            (double) (robotOdometry_->right_wheel_est_vel_ + robotOdometry_->left_wheel_est_vel_) / 2;
}

void Kinematics::estimateAngularVelocity()
{
    robotOdometry_->angular_est_vel_ =
            (double) (robotOdometry_->right_wheel_est_vel_ - robotOdometry_->left_wheel_est_vel_) / robotParamaters_->wheelSeparation_;
}

void Kinematics::setLeftWheelEstVel(const double& velocity)
{
    robotOdometry_->left_wheel_est_vel_ = velocity;
}

void Kinematics::setRightWheelEstVel(const double& velocity)
{
    robotOdometry_->right_wheel_est_vel_ = velocity;
}

void Kinematics::estimatePosition(const double& deltaTime)
{
    estimateAngularVelocity();
    estimateLinearVelocity();

    robotOdometry_->velocity_est_x =
            robotOdometry_->linear_est_vel_ * cos(robotOdometry_->theta_est_pose_);
    robotOdometry_->velocity_est_y =
            robotOdometry_->linear_est_vel_ * sin(robotOdometry_->theta_est_pose_);

    robotOdometry_->x_est_pose_ += robotOdometry_->velocity_est_x * deltaTime;
    robotOdometry_->y_est_pose_ += robotOdometry_->velocity_est_y * deltaTime;
    robotOdometry_->theta_est_pose_ +=
            robotOdometry_->angular_est_vel_ * deltaTime;
}

double Kinematics::getXEstPose() { return robotOdometry_->x_est_pose_; }

double Kinematics::getYEstPose() { return robotOdometry_->y_est_pose_; }

double Kinematics::getThetaEstPose() { return robotOdometry_->theta_est_pose_; }

double Kinematics::getVelocityEstX() { return robotOdometry_->velocity_est_x; }

double Kinematics::getVelocityEstY() { return robotOdometry_->velocity_est_y; }

double Kinematics::getVelocityEstTheta()
{
    return robotOdometry_->angular_est_vel_;
}
}// namespace rover_odometry
