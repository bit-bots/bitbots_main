#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_PYWRAPPER_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_PYWRAPPER_H_

#include <iostream>
#include <map>
#include <Python.h>
#include <Eigen/Geometry>
#include <boost/python.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <bitbots_dynamic_kick/kick_node.h>

class PyKickWrapper {
 public:
  explicit PyKickWrapper(std::string ns);
  moveit::py_bindings_tools::ByteString step(double dt, const std::string &joint_state_str);
  bool set_goal(const std::string &goal_str, const std::string &trunk_to_base_footprint_str);
  double get_progress();
  void set_params(boost::python::object params);

 private:
  std::shared_ptr<bitbots_dynamic_kick::KickNode> kick_node_;
};

#endif //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_PYWRAPPER_H_
