#ifndef BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H
#define BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H

#include <combined_robot_hw/CombinedRobotHW.h>


namespace bitbots_ros_control {

class WolfgangHardwareInterface: public combined_robot_hw::CombinedRobotHW{
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);

};

#endif //BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H
