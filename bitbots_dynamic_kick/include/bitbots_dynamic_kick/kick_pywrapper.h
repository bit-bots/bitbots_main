#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_PYWRAPPER_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_PYWRAPPER_H_

#include <iostream>
#include <map>
#include <Python.h>
#include <boost/python.hpp>
#include <ros/ros.h>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <bitbots_dynamic_kick/kick_node.h>

class PyKickWrapper {
 public:
  PyKickWrapper(const std::string ns);
  moveit::py_bindings_tools::ByteString step(double dt);
  bool init(const std::string &goal);
  double get_progress();
  void set_params(const boost::python::object params);

 private:
  std::shared_ptr<bitbots_dynamic_kick::KickNode> kick_node_;
};

#endif //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_PYWRAPPER_H_
