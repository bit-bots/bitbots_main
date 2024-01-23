#ifndef BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
#define BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <bitbots_msgs/msg/foot_pressure.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <bitbots_msgs/msg/robot_control_state.hpp>
#include <cmath>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <ros2_python_extension/serialization.hpp>

#include "bitbots_quintic_walk/walk_node.hpp"
#include "bitbots_quintic_walk/walk_utils.hpp"

namespace py = pybind11;
using namespace ros2_python_extension;

class PyWalkWrapper {
 public:
  explicit PyWalkWrapper(std::string ns, std::vector<py::bytes> parameter_msgs = {},
                         bool force_smooth_step_transition = false);
  py::bytes step(double dt, py::bytes &cmdvel_msg, py::bytes &imu_msg, py::bytes &jointstate_msg,
                 py::bytes &pressure_left, py::bytes &pressure_right);
  py::bytes step_relative(double dt, py::bytes &step_msg, py::bytes &imu_msg, py::bytes &jointstate_msg,
                          py::bytes &pressure_left, py::bytes &pressure_right);
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
  py::bytes get_support_state();
  bool is_left_support();
  void spin_some();
  void publish_debug();
  bool reset_and_test_if_speed_possible(py::bytes cmd_vel, double pos_threshold);

 private:
  std::shared_ptr<bitbots_quintic_walk::WalkNode> walk_node_;
};

#endif  // BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
