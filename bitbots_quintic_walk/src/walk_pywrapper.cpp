#include "bitbots_quintic_walk/walk_pywrapper.h"

void PyWalkWrapper::spin_some() {
  rclcpp::spin_some(walk_node_);
}

PyWalkWrapper::PyWalkWrapper(std::string ns) : walk_node_(std::make_shared<bitbots_quintic_walk::WalkNode>(ns)) {
  set_robot_state(0);
  walk_node_->initializeEngine();
}

py::bytes PyWalkWrapper::step(double dt,
                              py::bytes &cmdvel_msg,
                              py::bytes &imu_msg,
                              py::bytes &jointstate_msg,
                              py::bytes &pressure_left,
                              py::bytes &pressure_right) {
  bitbots_msgs::msg::JointCommand result = walk_node_->step(dt,
                                                            std::make_shared<geometry_msgs::msg::Twist>(fromPython<
                                                                geometry_msgs::msg::Twist>(cmdvel_msg)),
                                                            std::make_shared<sensor_msgs::msg::Imu>(fromPython<
                                                                sensor_msgs::msg::Imu>(imu_msg)),
                                                            std::make_shared<sensor_msgs::msg::JointState>(fromPython<
                                                                sensor_msgs::msg::JointState>(jointstate_msg)),
                                                            std::make_shared<bitbots_msgs::msg::FootPressure>(fromPython<
                                                                bitbots_msgs::msg::FootPressure>(pressure_left)),
                                                            std::make_shared<bitbots_msgs::msg::FootPressure>(fromPython<
                                                                bitbots_msgs::msg::FootPressure>(pressure_right)));
  return toPython<bitbots_msgs::msg::JointCommand>(result);
}

py::bytes PyWalkWrapper::step_open_loop(double dt, py::bytes &cmdvel_msg) {
  geometry_msgs::msg::PoseArray result = walk_node_->step_open_loop(dt,
                                                                    std::make_shared<geometry_msgs::msg::Twist>(
                                                                        fromPython<geometry_msgs::msg::Twist>(cmdvel_msg)));
  return toPython<geometry_msgs::msg::PoseArray>(result);
}

py::bytes PyWalkWrapper::get_left_foot_pose() {
  geometry_msgs::msg::Pose result = walk_node_->get_left_foot_pose();
  return toPython<geometry_msgs::msg::Pose>(result);
}
py::bytes PyWalkWrapper::get_right_foot_pose() {
  geometry_msgs::msg::Pose result = walk_node_->get_right_foot_pose();
  return toPython<geometry_msgs::msg::Pose>(result);
}

py::bytes PyWalkWrapper::get_odom() {
  nav_msgs::msg::Odometry result = walk_node_->getOdometry();
  return toPython<nav_msgs::msg::Odometry>(result);
}

void PyWalkWrapper::reset() {
  walk_node_->reset();
}

void PyWalkWrapper::special_reset(int state, double phase, py::bytes cmd_vel, bool reset_odometry) {
  bitbots_quintic_walk::WalkState walk_state;
  if (state == 0) {
    walk_state = bitbots_quintic_walk::WalkState::PAUSED;
  } else if (state == 1) {
    walk_state = bitbots_quintic_walk::WalkState::WALKING;
  } else if (state == 2) {
    walk_state = bitbots_quintic_walk::WalkState::IDLE;
  } else if (state == 3) {
    walk_state = bitbots_quintic_walk::WalkState::START_MOVEMENT;
  } else if (state == 4) {
    walk_state = bitbots_quintic_walk::WalkState::STOP_MOVEMENT;
  } else if (state == 5) {
    walk_state = bitbots_quintic_walk::WalkState::START_STEP;
  } else if (state == 6) {
    walk_state = bitbots_quintic_walk::WalkState::STOP_STEP;
  } else if (state == 7) {
    walk_state = bitbots_quintic_walk::WalkState::KICK;
  } else {
    RCLCPP_WARN(walk_node_->get_logger(), "state in special reset not clear");
    return;
  }
  walk_node_->reset(walk_state,
                    phase,
                    std::make_shared<geometry_msgs::msg::Twist>(fromPython<geometry_msgs::msg::Twist>(cmd_vel)),
                    reset_odometry);
}

double PyWalkWrapper::get_phase() {
  return walk_node_->getEngine()->getPhase();
}

double PyWalkWrapper::get_freq() {
  return walk_node_->getEngine()->getFreq();
}

void PyWalkWrapper::set_robot_state(int state) {
  humanoid_league_msgs::msg::RobotControlState state_msg;
  state_msg.state = state;
  walk_node_->robotStateCb(std::make_shared<humanoid_league_msgs::msg::RobotControlState>(state_msg));
}

bool string2bool(std::string &v) {
  return !v.empty() &&
      (strcasecmp(v.c_str(), "true") == 0 ||
          atoi(v.c_str()) != 0);
}

void PyWalkWrapper::set_parameters(py::dict params) {
  std::vector<rclcpp::Parameter> parameters;
  for (auto item: params) {
    rclcpp::Parameter parameter =
        rclcpp::Parameter(item.first.cast<std::string>(), rclcpp::ParameterValue(item.second.cast<std::string>()));
    parameters.push_back(parameter);
  };

  walk_node_->onSetParameters(parameters);
}

PYBIND11_MODULE(libpy_quintic_walk, m) {
  using namespace bitbots_quintic_walk;

  m.def("initRos", &ros2_python_extension::initRos);

  py::class_<PyWalkWrapper, std::shared_ptr<PyWalkWrapper>>(m, "PyWalkWrapper")
      .def(py::init<std::string>())
      .def("step", &PyWalkWrapper::step)
      .def("step_open_loop", &PyWalkWrapper::step_open_loop)
      .def("get_left_foot_pose", &PyWalkWrapper::get_left_foot_pose)
      .def("get_right_foot_pose", &PyWalkWrapper::get_right_foot_pose)
      .def("set_robot_state", &PyWalkWrapper::set_robot_state)
      .def("reset", &PyWalkWrapper::reset)
      .def("special_reset", &PyWalkWrapper::special_reset)
      .def("set_parameters", &PyWalkWrapper::set_parameters)
      .def("get_phase", &PyWalkWrapper::get_phase)
      .def("get_freq", &PyWalkWrapper::get_freq)
      .def("get_odom", &PyWalkWrapper::get_odom)
      .def("spin_some", &PyWalkWrapper::spin_some);
}