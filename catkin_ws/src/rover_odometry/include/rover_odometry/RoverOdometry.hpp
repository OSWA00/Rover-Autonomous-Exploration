#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "rover_odometry/Kinematics.hpp"

namespace rover_odometry {
class RoverOdometry {
   public:
    RoverOdometry(ros::NodeHandle& nodeHandle);
    virtual ~RoverOdometry();
    void publishOdom();

   private:
    bool readParameters();
    void wlCallback(const std_msgs::Float32& message);
    void wrCallback(const std_msgs::Float32& message);

    ros::NodeHandle& nodeHandle_;

    ros::Time timeCurrent_;
    ros::Time timeLast_;

    ros::Subscriber wl_;
    std::string wlTopic_;

    ros::Subscriber wr_;
    std::string wrTopic_;

    ros::Publisher odom_;
    std::string odomTopic_;
    std::string frameId_;
    std::string childFrameId_;

    tf2_ros::TransformBroadcaster odomBroadcaster_;
    geometry_msgs::TransformStamped odomTransform_;

    float wheelRadius_;
    float wheelSeparation_;

    Kinematics kinematics_;
};
}  // namespace rover_odometry
