#include "bitbots_dynup/dynup_pywrapper.h"

PyDynupWrapper::PyDynupWrapper(const std::string ns) : dynup_node_(std::make_shared<bitbots_dynup::DynupNode>(ns)){
}

void PyDynupWrapper::spin_some() {
    rclcpp::spin_some(dynup_node_);
}

py::bytes PyDynupWrapper::step(double dt,
                               py::bytes &imu_msg,
                               py::bytes &jointstate_msg) {
    bitbots_msgs::msg::JointCommand result =
        dynup_node_->step(dt,
             std::make_shared<sensor_msgs::msg::Imu>(fromPython<sensor_msgs::msg::Imu>(imu_msg)),
             std::make_shared<sensor_msgs::msg::JointState>(fromPython<sensor_msgs::msg::JointState>(jointstate_msg)));
    return toPython<bitbots_msgs::msg::JointCommand>(result);
}

py::bytes PyDynupWrapper::step_open_loop(double dt) {
    geometry_msgs::msg::PoseArray result = dynup_node_->step_open_loop(dt);
    return toPython<geometry_msgs::msg::PoseArray>(result);
}

void PyDynupWrapper::reset() {
    dynup_node_->reset();
}

void PyDynupWrapper::special_reset(double time) {
    dynup_node_->reset(time);
}

void PyDynupWrapper::set_engine_goal(std::string direction) {
    bitbots_dynup::msg::DynupPoses poses = dynup_node_->getCurrentPoses();
    if (!poses.header.stamp.sec == 0) {
        DynupRequest request;
        request.direction = direction;
        request.l_foot_pose = poses.l_leg_pose;
        request.r_foot_pose = poses.r_leg_pose;
        request.l_hand_pose = poses.l_arm_pose;
        request.r_hand_pose = poses.r_arm_pose;
        dynup_node_->getEngine()->setGoals(request);
        dynup_node_->getIK()->setDirection(request.direction);
    }
}

int PyDynupWrapper::get_direction() {
    return dynup_node_->getEngine()->getDirection();
}

py::bytes PyDynupWrapper::get_poses() {
    bitbots_dynup::msg::DynupPoses poses = dynup_node_->getCurrentPoses();
    return toPython<bitbots_dynup::msg::DynupPoses>(poses);
}

bool string2bool(const std::string &v) {
    return !v.empty() &&
           (strcasecmp(v.c_str(), "true") == 0 ||
            atoi(v.c_str()) != 0);
}

void PyDynupWrapper::set_parameter(py::bytes parameter_msg) {
    // convert serialized parameter msg to parameter object
    rclcpp::Parameter
        parameter = rclcpp::Parameter::from_parameter_msg(fromPython<rcl_interfaces::msg::Parameter>(parameter_msg));

    // needs to be a vector
    std::vector<rclcpp::Parameter> parameters = {parameter};
    dynup_node_->onSetParameters(parameters);
}


PYBIND11_MODULE(libpy_dynup, m)
    {
        using namespace bitbots_dynup;

        m.def("initRos", &ros2_python_extension::initRos);

        py::class_<PyDynupWrapper, std::shared_ptr<PyDynupWrapper>>(m, "PyDynupWrapper")
        .def(py::init<std::string, std::vector<py::bytes>>())
        .def("step", &PyDynupWrapper::step)
        .def("step_open_loop", &PyDynupWrapper::step_open_loop)
        .def("reset", &PyDynupWrapper::reset)
        .def("special_reset", &PyDynupWrapper::special_reset)
        .def("set_parameter", &PyDynupWrapper::set_parameter)
        .def("get_poses", &PyDynupWrapper::get_poses)
        .def("get_direction", &PyDynupWrapper::get_direction)
        .def("set_engine_goal", &PyDynupWrapper::set_engine_goal)
        .def("spin_some", &PyDynupWrapper::spin_some);
    }