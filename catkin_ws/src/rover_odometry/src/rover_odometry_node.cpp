#include <ros/ros.h>

#include "rover_odometry/RoverOdometry.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rover_odometry");
    ros::NodeHandle nodeHandle("~");

    rover_odometry::RoverOdometry roverOdometry(nodeHandle);

    ros::spin();
    return 0;
}
