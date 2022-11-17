#include "rover_odometry/RoverOdometry.hpp"

namespace rover_odometry {
RoverOdometry::RoverOdometry(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle) {
    if (!readParameters()) {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }
    wl_ = nodeHandle_.subscribe(wlTopic_, 1, &RoverOdometry::wlCallback, this);
    wr_ = nodeHandle_.subscribe(wrTopic_, 1, &RoverOdometry::wrCallback, this);
    odom_ = nodeHandle_.advertise<nav_msgs::Odometry>(odomTopic_, 50);

    kinematics_.addRobotParameters(wheelSeparation_, wheelRadius_);

    timeLast_ = ros::Time::now();

    publishCameraLink();

    ROS_INFO("Successfully launched node.");
}

RoverOdometry::~RoverOdometry() {}

bool RoverOdometry::readParameters() {
    if (!nodeHandle_.getParam("wl_topic", wlTopic_)) return false;
    if (!nodeHandle_.getParam("wr_topic", wrTopic_)) return false;
    if (!nodeHandle_.getParam("odom_topic", odomTopic_)) return false;
    if (!nodeHandle_.getParam("wheel_radius", wheelRadius_)) return false;
    if (!nodeHandle_.getParam("wheel_separation", wheelSeparation_))
        return false;
    if (!nodeHandle_.getParam("odom_frame", odomFrame_)) return false;
    if (!nodeHandle_.getParam("base_frame", baseFrame_)) return false;
    if (!nodeHandle_.getParam("camera_frame", cameraFrame_)) return false;

    return true;
}

void RoverOdometry::wlCallback(const std_msgs::Float32& message) {
    double angularVelocity = message.data;
    double filteredAngularVelocity =
        leftWheelFilter_.filterWheelAngularVelocity(angularVelocity);
    double velocity =
        kinematics_.estimateWheelLinearVelocity(filteredAngularVelocity);
    ROS_DEBUG("Left wheel velocity: %f", velocity);
    kinematics_.setLeftWheelEstVel(velocity);
}

void RoverOdometry::wrCallback(const std_msgs::Float32& message) {
    double angularVelocity = message.data;
    double filteredAngularVelocity =
        rightWheelFilter_.filterWheelAngularVelocity(angularVelocity);
    double velocity =
        kinematics_.estimateWheelLinearVelocity(filteredAngularVelocity);
    ROS_DEBUG("Right wheel velocity: %f", velocity);
    kinematics_.setRightWheelEstVel(velocity);
}

void RoverOdometry::publishOdom() {
    timeCurrent_ = ros::Time::now();
    double timeDelta = (timeCurrent_ - timeLast_).toSec();
    kinematics_.estimatePosition(timeDelta);
    timeLast_ = timeCurrent_;

    double velocityX = kinematics_.getVelocityEstX();
    double velocityY = kinematics_.getVelocityEstY();
    double velocityTheta = kinematics_.getVelocityEstTheta();

    double poseX = kinematics_.getXEstPose();
    double poseY = kinematics_.getYEstPose();
    double poseTheta = kinematics_.getThetaEstPose();

    tf2::Quaternion odomQuaternion;
    odomQuaternion.setRPY(0, 0, poseTheta);

    ROS_DEBUG("Theta %f", poseTheta);
    ROS_DEBUG("Quaternion: Z: %f  W: %f", odomQuaternion.getZ(),
              odomQuaternion.getW());

    geometry_msgs::Quaternion odomMessage;
    tf2::convert(odomQuaternion, odomMessage);

    odomTransform_.header.stamp = ros::Time::now();
    odomTransform_.header.frame_id = odomFrame_;
    odomTransform_.child_frame_id = baseFrame_;

    odomTransform_.transform.translation.x = poseX;
    odomTransform_.transform.translation.y = poseY;
    odomTransform_.transform.translation.z = 0.048;

    odomTransform_.transform.rotation = odomMessage;

    odomBroadcaster_.sendTransform(odomTransform_);

    nav_msgs::Odometry odom;
    odom.header.stamp = timeCurrent_;
    odom.header.frame_id = odomFrame_;

    odom.pose.pose.position.x = poseX;
    odom.pose.pose.position.y = poseY;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation = odomMessage;

    odom.child_frame_id = baseFrame_;

    odom.twist.twist.linear.x = velocityX;
    odom.twist.twist.linear.y = velocityY;
    odom.twist.twist.angular.z = velocityTheta;

    odom_.publish(odom);
}

void RoverOdometry::publishCameraLink() {
    cameraLinkTransform_.header.stamp = ros::Time::now();
    cameraLinkTransform_.header.frame_id = baseFrame_;
    cameraLinkTransform_.child_frame_id = cameraFrame_;

    cameraLinkTransform_.transform.translation.x = -0.2002;
    cameraLinkTransform_.transform.translation.y = 0.0;
    cameraLinkTransform_.transform.translation.z = 0.15178;

    tf2::Quaternion cameraQuat;
    cameraQuat.setRPY(0.0, 0.0, 0.0);

    cameraLinkTransform_.transform.rotation.x = cameraQuat.x();
    cameraLinkTransform_.transform.rotation.y = cameraQuat.y();
    cameraLinkTransform_.transform.rotation.z = cameraQuat.z();
    cameraLinkTransform_.transform.rotation.w = cameraQuat.w();

    cameraLinkBroadcaster_.sendTransform(cameraLinkTransform_);
    ROS_INFO("Camera link set");
}

}  // namespace rover_odometry