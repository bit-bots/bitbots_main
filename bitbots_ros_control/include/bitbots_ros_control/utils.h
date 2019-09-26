#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_UTILS_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_UTILS_H_

#include "ros/ros.h"
#include "humanoid_league_msgs/Speak.h"

namespace bitbots_ros_control {
    void speakError(const ros::Publisher &speak_pub, std::string text);
    uint16_t dxlMakeword(uint64_t a, uint64_t b);
    uint32_t dxlMakedword(uint64_t a, uint64_t b);
}

#endif  //BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_UTILS_H_
