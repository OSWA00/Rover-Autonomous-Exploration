#include <ros/ros.h>

#include "rover_odometry/RoverOdometry.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "rover_odometry");
    ros::NodeHandle nodeHandle("~");

    rover_odometry::RoverOdometry roverOdometry(nodeHandle);
    ros::Rate rate(400);

    while (ros::ok()) {
        roverOdometry.publishOdom();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
