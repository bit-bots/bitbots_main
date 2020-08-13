#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {

bool stringToControlMode(std::string _control_modestr, ControlMode &control_mode) {
  /**
   * Helper method to parse strings to corresponding control modes
   */
  if (_control_modestr == "position") {
    control_mode = POSITION_CONTROL;
    return true;
  } else if (_control_modestr == "velocity") {
    control_mode = VELOCITY_CONTROL;
    return true;
  } else if (_control_modestr == "effort") {
    control_mode = EFFORT_CONTROL;
    return true;
  } else if (_control_modestr == "current_based") {
    control_mode = CURRENT_BASED_POSITION_CONTROL;
    return true;
  } else {
    ROS_WARN("Trying to set unknown control mode");
    return false;
  }
}

void speakError(const ros::Publisher &speak_pub, std::string text) {
  /**
    *  Helper method to send a message for text-to-speech output
    */
  humanoid_league_msgs::Speak msg = humanoid_league_msgs::Speak();
  msg.text = text;
  msg.priority = humanoid_league_msgs::Speak::HIGH_PRIORITY;
  speak_pub.publish(msg);
}

uint16_t dxlMakeword(uint64_t a, uint64_t b) {
  return uint16_t(uint8_t(a & 0xff) | uint16_t(uint8_t(b & 0xff)) << 8);
}

uint32_t dxlMakedword(uint64_t a, uint64_t b) {
  return uint32_t(uint16_t(a & 0xffff) | uint32_t(uint16_t(b & 0xffff) << 16));
}

float dxlMakeFloat(uint8_t *data) {
  float f;
  uint32_t b = dxlMakedword(dxlMakeword(data[0], data[1]), dxlMakeword(data[2], data[3]));
  memcpy(&f, &b, sizeof(f));
  return f;
}

std::string gyroRangeToString(uint8_t range) {
  switch (range) {
    case 0:return "250 deg/s";
    case 1:return "500 deg/s";
    case 2:return "1000 deg/s";
    case 3:return "2000 deg/s";
    default:return "invalid range, defaulting to 2000 deg/s";
  }
}

std::string accelRangeToString(uint8_t range) {
  switch (range) {
    case 0:return "2G";
    case 1:return "4G";
    case 2:return "8G";
    case 3:return "16G";
    default:return "invalid range, defaulting to 16G";
  }
}
}
