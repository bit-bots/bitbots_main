#ifndef BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
#define BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
#include "bitbots_quintic_walk/walk_node.h"
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <iostream>
#include "bitbots_quintic_walk/walk_utils.h"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <bitbots_msgs/msg/foot_pressure.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <humanoid_league_msgs/msg/robot_control_state.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ros2_python_extension/init.hpp>
#include <ros2_python_extension/serialization.hpp>

namespace py = pybind11;
using namespace ros2_python_extension;

class PyWalkWrapper {
 public:
  PyWalkWrapper(std::string ns, std::vector<py::bytes> parameter_msgs = {});
  py::bytes step(double dt,
                 py::bytes &cmdvel_msg,
                 py::bytes &imu_msg,
                 py::bytes &jointstate_msg,
                 py::bytes &pressure_left,
                 py::bytes &pressure_right);
  py::bytes step_relative(double dt,
                          py::bytes &step_msg,
                          py::bytes &imu_msg,
                          py::bytes &jointstate_msg,
                          py::bytes &pressure_left,
                          py::bytes &pressure_right);
  py::bytes step_open_loop(double dt, py::bytes &cmdvel_msg);
  py::bytes get_left_foot_pose();
  py::bytes get_right_foot_pose();
  py::bytes get_odom();
  void reset();
  void special_reset(int state, double phase, py::bytes cmd_vel, bool reset_odometry);
  void set_robot_state(int state);
  void set_parameter(const py::bytes parameter_msg);
  double get_phase();
  double get_freq();
  void spin_some();

 private:
  std::shared_ptr<bitbots_quintic_walk::WalkNode> walk_node_;
};

#endif //BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
