#pragma once

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

#include "rover_odometry/Kinematics.hpp"

namespace rover_odometry {
class RoverOdometry {
   public:
    RoverOdometry(ros::NodeHandle& nodeHandle);
    virtual ~RoverOdometry();

   private:
    bool readParameters();
    void wlCallback(const std_msgs::Float32& message);
    void wrCallback(const std_msgs::Float32& message);

    ros::NodeHandle& nodeHandle_;

    ros::Subscriber wl_;
    std::string wlTopic_;

    ros::Subscriber wr_;
    std::string wrTopic_;

    ros::Publisher odom_;
    std::string odomTopic_;

    Kinematics kinematics_;
};
}  // namespace rover_odometry
