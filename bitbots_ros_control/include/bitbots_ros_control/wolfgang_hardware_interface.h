#ifndef BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H
#define BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H

#include <bitbots_ros_control/imu_hardware_interface.h>
#include <bitbots_ros_control/dynamixel_servo_hardware_interface.h>
#include <bitbots_ros_control/bitfoot_hardware_interface.h>
#include <bitbots_ros_control/button_hardware_interface.h>
#include <hardware_interface/robot_hw.h>

namespace bitbots_ros_control {

void speak(const ros::Publisher& speak_pub, std::string text){
  /**
   *  Helper method to send a message for text-to-speech output
   */
  humanoid_league_msgs::Speak msg = humanoid_league_msgs::Speak();
  msg.text = text;
  msg.priority = humanoid_league_msgs::Speak::HIGH_PRIORITY;
  speak_pub.publish(msg);
}


class WolfgangHardwareInterface : public hardware_interface::RobotHW {
public:
    WolfgangHardwareInterface(ros::NodeHandle& nh);
    bool init(ros::NodeHandle &nh);

    bool read();

    void write();

private:
    // in the future we will have servos on multiple buses, but currently just one
    DynamixelServoHardwareInterface _servos;

    ImuHardwareInterface _imu;
    BitFootHardwareInterface _left_foot;
    BitFootHardwareInterface _right_foot;
    ButtonHardwareInterface _buttons;

    ros::Publisher _speak_pub;
};
}

#endif //BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H
