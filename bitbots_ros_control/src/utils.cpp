#include "bitbots_ros_control/utils.h"

namespace bitbots_ros_control {

    void speak_error(const ros::Publisher &speak_pub, std::string text) {
/**
 *  Helper method to send a message for text-to-speech output
 */
      humanoid_league_msgs::Speak msg = humanoid_league_msgs::Speak();
      msg.text = text;
      msg.priority = humanoid_league_msgs::Speak::HIGH_PRIORITY;
      speak_pub.publish(msg);
    }
}