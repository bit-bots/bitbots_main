//
// Created by nfiedler on 6/13/20.
//

#ifndef BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
#define BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
#include <Python.h>
#include "bitbots_quintic_walk/walk_node.h"
#include <boost/python.hpp>
#include <ros/ros.h>
#include <map>
#include <iostream>

class PyWalkWrapper {
 public:
  PyWalkWrapper();
  std::string step(double dt, const std::string &cmdvel_msg, const std::string &imu_msg, const std::string &jointstate_msg);
  void reset();
  void set_robot_state(int state);

 private:
  std::shared_ptr<bitbots_quintic_walk::WalkNode> walk_node_;
};



#endif //BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
