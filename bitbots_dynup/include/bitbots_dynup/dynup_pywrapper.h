#ifndef BITBOTS_META_DYNUP_PYWRAPPER_H
#define BITBOTS_META_DYNUP_PYWRAPPER_H

#include <Python.h>
#include "bitbots_dynup/dynup_node.h"
#include <boost/python.hpp>
#include <ros/ros.h>
#include <map>
#include <iostream>
#include <moveit/py_bindings_tools/serialize_msg.h>
#include <bitbots_dynup/DynUpConfig.h>
#include "bitbots_dynup/dynup_utils.h"

class PyDynupWrapper {
public:
    PyDynupWrapper(const std::string ns);
    moveit::py_bindings_tools::ByteString step(double dt,
                                               const std::string &cmdvel_msg,
                                               const std::string &imu_msg,
                                               const std::string &jointstate_msg);
    moveit::py_bindings_tools::ByteString get_poses();
    void reset();
    void special_reset(double time);
    int get_direction();
    void set_node_dyn_reconf(const boost::python::object params);

private:
    std::shared_ptr<bitbots_dynup::DynupNode> dynup_node_;
};

#endif //BITBOTS_META_DYNUP_PYWRAPPER_H

