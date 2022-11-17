#pragma once

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace rover_control {
class RoverControl {
   private:
    ros::NodeHandle nodeHandle_;
    nav_msgs::Odometry odom_;
    geometry_msgs::Pose target_;

   public:
    RoverControl(ros::NodeHandle& nodeHandle);
    virtual ~RoverControl();
    void odomCallback(const nav_msgs::Odometry& odom);
    bool setPoseCallback(std_srvs::Trigger::Request& request,
                         std_srvs::Trigger::Response& response);
};
}  // namespace rover_control