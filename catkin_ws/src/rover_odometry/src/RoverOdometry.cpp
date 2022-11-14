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

    kinematics_.addRobotParameters(wheelRadius_, wheelSeparation_);

    timeLast_ = ros::Time::now();

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
    if (!nodeHandle_.getParam("odom_frame", frameId_)) return false;
    if (!nodeHandle_.getParam("base_frame", childFrameId_)) return false;

    return true;
}

void RoverOdometry::wlCallback(const std_msgs::Float32& message) {
    float angularVelocity = message.data;
    float filteredAngularVelocity = leftWheelFilter_.filterWheelAngularVelocity(angularVelocity);
    float velocity = kinematics_.estimateWheelLinearVelocity(filteredAngularVelocity);
    kinematics_.setLeftWheelEstVel(velocity);
}

void RoverOdometry::wrCallback(const std_msgs::Float32& message) {
    float angularVelocity = message.data;
    float filteredAngularVelocity = rightWheelFilter_.filterWheelAngularVelocity(angularVelocity);
    float velocity = kinematics_.estimateWheelLinearVelocity(filteredAngularVelocity);
    kinematics_.setRightWheelEstVel(velocity);
}

void RoverOdometry::publishOdom() {
    timeCurrent_ = ros::Time::now();
    float timeDelta = (timeCurrent_ - timeLast_).toSec();
    kinematics_.estimatePosition(timeDelta);
    timeLast_ = timeCurrent_;

    float velocityX = kinematics_.getVelocityEstX();
    float velocityY = kinematics_.getVelocityEstY();
    float velocityTheta = kinematics_.getVelocityEstTheta();

    float poseX = kinematics_.getXEstPose();
    float poseY = kinematics_.getYEstPose();
    float poseTheta = kinematics_.getThetaEstPose();

    tf2::Quaternion odomQuaternion;
    odomQuaternion.setRPY(0, 0, poseTheta);

    ROS_DEBUG("Theta %f", poseTheta);
    ROS_DEBUG("Quaternion: Z: %f  W: %f", odomQuaternion.getZ(), odomQuaternion.getW());

    geometry_msgs::Quaternion odomMessage;
    tf2::convert(odomQuaternion, odomMessage);

    odomTransform_.header.stamp = ros::Time::now();
    odomTransform_.header.frame_id = frameId_;
    odomTransform_.child_frame_id = childFrameId_;

    odomTransform_.transform.translation.x = poseX;
    odomTransform_.transform.translation.y = poseY;
    odomTransform_.transform.translation.z = 0.048;

    odomTransform_.transform.rotation = odomMessage;

    odomBroadcaster_.sendTransform(odomTransform_);

    nav_msgs::Odometry odom;
    odom.header.stamp = timeCurrent_;
    odom.header.frame_id = frameId_;

    odom.pose.pose.position.x = poseX;
    odom.pose.pose.position.y = poseY;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation = odomMessage;

    odom.child_frame_id = childFrameId_;

    odom.twist.twist.linear.x = velocityX;
    odom.twist.twist.linear.y = velocityY;
    odom.twist.twist.angular.z = velocityTheta;

    odom_.publish(odom);
}

}  // namespace rover_odometry