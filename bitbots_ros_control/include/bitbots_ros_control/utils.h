#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_UTILS_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_UTILS_H_

#include "rclcpp/rclcpp.hpp"
#include "humanoid_league_msgs/msg/audio.hpp"

namespace bitbots_ros_control {

enum ControlMode {
  POSITION_CONTROL,
  VELOCITY_CONTROL,
  EFFORT_CONTROL,
  CURRENT_BASED_POSITION_CONTROL
};

bool stringToControlMode(rclcpp::Node::SharedPtr nh, std::string control_modestr, ControlMode &control_mode);
void speakError(rclcpp::Publisher<humanoid_league_msgs::msg::Audio>::SharedPtr speak_pub, std::string text);

uint16_t dxlMakeword(uint64_t a, uint64_t b);
uint32_t dxlMakedword(uint64_t a, uint64_t b);
float dxlMakeFloat(uint8_t *data);

std::string gyroRangeToString(uint8_t range);
std::string accelRangeToString(uint8_t range);
}

#endif  //BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_UTILS_H_
