#ifndef BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H
#define BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H

#include <bitbots_ros_control/imu_hardware_interface.h>
#include <bitbots_ros_control/dynamixel_servo_hardware_interface.h>
#include <bitbots_ros_control/bitfoot_hardware_interface.h>
#include <bitbots_ros_control/button_hardware_interface.h>

namespace bitbots_ros_control {

class WolfgangHardwareInterface: public hardware_interface::RobotHW{
public:
    bool init(ros::NodeHandle& nh);
    bool read();
    void write();

private:
    // in the future we will have servos on multiple buses
    DynamixelServoHardwareInterface _servo_bus1;

    ImuHardwareInterface _imu;
    BitFootHardwareInterface _left_foot;
    BitFootHardwareInterface _right_foot;
    ButtonHardwareInterface _buttons;
};

#endif //BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H
