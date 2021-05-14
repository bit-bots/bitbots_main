#ifndef BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
#define BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
#include <Python.h>
#include "bitbots_quintic_walk/walk_node.h"
#include <boost/python.hpp>
#include <ros/ros.h>
#include <map>
#include <iostream>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <bitbots_quintic_walk/bitbots_quintic_walk_engine_paramsConfig.h>
#include "bitbots_quintic_walk/walk_utils.h"

class PyWalkWrapper {
 public:
  PyWalkWrapper(const std::string ns);
  moveit::py_bindings_tools::ByteString step(double dt,
                                             const std::string &cmdvel_msg,
                                             const std::string &imu_msg,
                                             const std::string &jointstate_msg,
                                             const std::string &pressure_left,
                                             const std::string &pressure_right);
  moveit::py_bindings_tools::ByteString get_left_foot_pose();
  moveit::py_bindings_tools::ByteString get_odom();
  void reset();
  void special_reset(int state, double phase, const std::string cmd_vel, bool reset_odometry);
  void set_robot_state(int state);
  void set_engine_dyn_reconf(const boost::python::object params);
  void set_node_dyn_reconf(const boost::python::object params);
  float get_phase();
  float get_freq();

 private:
  std::shared_ptr<bitbots_quintic_walk::WalkNode> walk_node_;
};

#endif //BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
