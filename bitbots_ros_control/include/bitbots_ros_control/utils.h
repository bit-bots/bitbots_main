#ifndef BITBOTS_ROS_CONTROL_UTILS__H
#define BITBOTS_ROS_CONTROL_UTILS__H

#include "ros/ros.h"
#include "humanoid_league_msgs/Speak.h"

namespace bitbots_ros_control {

    void speak_error(const ros::Publisher &speak_pub, std::string text);
}
#endif