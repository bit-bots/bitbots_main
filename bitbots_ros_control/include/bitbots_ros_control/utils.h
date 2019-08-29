#ifndef BITBOTS_ROS_CONTROL_UTILS__H
#define BITBOTS_ROS_CONTROL_UTILS__H

#include "ros/ros.h"
#include "humanoid_league_msgs/Speak.h"

namespace bitbots_ros_control {
    void speak_error(const ros::Publisher &speak_pub, std::string text);
    uint16_t dxl_makeword(uint64_t a, uint64_t b);
    uint32_t dxl_makedword(uint64_t a, uint64_t b);
}

#endif  // BITBOTS_ROS_CONTROL_UTILS__H
