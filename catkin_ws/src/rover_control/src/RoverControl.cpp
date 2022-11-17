#include "rover_control/RoverControl.hpp"

namespace rover_control {
RoverControl::RoverControl(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle) {
    ROS_INFO("Succesfully launched node.");
}

RoverControl::~RoverControl(){};

void RoverControl::odomCallback(const nav_msgs::Odometry& odom) {
    odom_ = odom;
}

bool RoverControl::setPoseCallback(std_srvs::Trigger::Request& request,
                                   std_srvs::Trigger::Response& response) {
response.success = true;
response.message = "Goal pose set";
return true;
}

}  // namespace rover_control
