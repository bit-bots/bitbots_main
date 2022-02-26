#ifndef BITBOTS_DYNUP_BITBOTS_DYNUP_SRC_DYNUP_PYWRAPPER_H
#define BITBOTS_DYNUP_BITBOTS_DYNUP_SRC_DYNUP_PYWRAPPER_H

#include "bitbots_dynup/dynup_node.h"
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <iostream>
#include "bitbots_dynup/dynup_utils.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ros2_python_extension/init.hpp>
#include <ros2_python_extension/serialization.hpp>

namespace py = pybind11;
using namespace ros2_python_extension;

class PyDynupWrapper {
public:
    PyDynupWrapper(std::string ns);
    moveit::py_bindings_tools::ByteString step(double dt,
                                               const std::string &imu_msg,
                                               const std::string &jointstate_msg);
    moveit::py_bindings_tools::ByteString step_open_loop(double dt);
    moveit::py_bindings_tools::ByteString get_poses();
    void reset();
    void special_reset(double time);
    void set_engine_goal(std::string direction);
    int get_direction();
    void set_node_dyn_reconf(const boost::python::object params);

private:
    std::shared_ptr<bitbots_dynup::DynupNode> dynup_node_;
};

#endif //BITBOTS_DYNUP_BITBOTS_DYNUP_SRC_DYNUP_PYWRAPPER_H

