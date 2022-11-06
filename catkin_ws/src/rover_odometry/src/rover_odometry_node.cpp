#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rover_odometry");
  ros::NodeHandle nodeHandle("~");

  ros::spin();
  return 0;
}
