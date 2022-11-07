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

    ROS_INFO("Successfully launched node.");
}

RoverOdometry::~RoverOdometry() {}

bool RoverOdometry::readParameters() {
    if (!nodeHandle_.getParam("wl_topic", wlTopic_)) return false;
    if (!nodeHandle_.getParam("wr_topic", wrTopic_)) return false;
    if (!nodeHandle_.getParam("odom_topic", odomTopic_)) return false;
    return true;
}

void RoverOdometry::wlCallback(const std_msgs::Float32& message) {}

void RoverOdometry::wrCallback(const std_msgs::Float32& message) {}

}  // namespace rover_odometry