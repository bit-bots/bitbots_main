#ifndef BITFOOT_HARWARE_INTERFACE_H
#define BITFOOT_HARWARE_INTERFACE_H

#include <ros/ros.h>
#include <string>

#include <humanoid_league_msgs/Speak.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <bitbots_msgs/FootPressure.h>

#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/server.h>

#include <bitbots_ros_control/bitbots_ros_control_paramsConfig.h>

#include <dynamixel_workbench/dynamixel_driver.h>

namespace bitbots_ros_control {

class BitFootHardwareInterface : public hardware_interface::RobotHW {
public:
    BitFootHardwareInterface();

    bool init(ros::NodeHandle &nh);

    bool read();

    void write();

    void set_driver(boost::shared_ptr<DynamixelDriver> driver);

private:
    ros::NodeHandle _nh;

    boost::shared_ptr<DynamixelDriver> _driver;

    std::vector<double> _current_pressure;

    ros::Publisher _pressure_pub;

};
}
#endif
