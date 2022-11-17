#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "rover_odometry/FIRFilter.hpp"
#include "rover_odometry/Kinematics.hpp"

namespace rover_odometry {
class RoverOdometry {
   public:
    RoverOdometry(ros::NodeHandle& nodeHandle);
    virtual ~RoverOdometry();
    void publishOdom();
    void publishCameraLink();

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
    std::string odomFrame_;
    std::string baseFrame_;
    std::string cameraFrame_;

    tf2_ros::TransformBroadcaster odomBroadcaster_;
    geometry_msgs::TransformStamped odomTransform_;

    tf2_ros::StaticTransformBroadcaster cameraLinkBroadcaster_;
    geometry_msgs::TransformStamped cameraLinkTransform_;

    double wheelRadius_;
    double wheelSeparation_;

    Kinematics kinematics_;

    FIRFilter leftWheelFilter_;
    FIRFilter rightWheelFilter_;
};
}  // namespace rover_odometry
