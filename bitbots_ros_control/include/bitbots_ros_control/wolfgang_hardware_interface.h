#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_

#include <bitbots_ros_control/imu_hardware_interface.h>
#include <bitbots_ros_control/dynamixel_servo_hardware_interface.h>
#include <bitbots_ros_control/bitfoot_hardware_interface.h>
#include <bitbots_ros_control/button_hardware_interface.h>
#include <hardware_interface/robot_hw.h>


namespace bitbots_ros_control {

class WolfgangHardwareInterface : public hardware_interface::RobotHW {
public:
    explicit WolfgangHardwareInterface(ros::NodeHandle& nh);
    bool init(ros::NodeHandle &nh);

    bool read();

    void write();

private:
    // in the future we will have servos on multiple buses, but currently just one
    DynamixelServoHardwareInterface servos_;

    ImuHardwareInterface imu_;
    BitFootHardwareInterface left_foot_;
    BitFootHardwareInterface right_foot_;
    ButtonHardwareInterface buttons_;

    ros::Publisher speak_pub_;

    bool only_imu_;
    bool only_pressure_;
};
}

#endif //BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_
