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

class PyWalk {
 public:
  PyWalk();
  int step(int);

 private:
  std::shared_ptr<bitbots_quintic_walk::WalkNode> walk_node_;
};



#endif //BITBOTS_QUINTIC_WALK_BITBOTS_QUINTIC_WALK_SRC_WALK_PYWRAPPER_H_
