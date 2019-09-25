#include <bitbots_ros_control/utils.h>

namespace bitbots_ros_control {

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
}
