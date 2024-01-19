#include "bitbots_quintic_walk/walk_pywrapper.hpp"

void PyWalkWrapper::spin_some() { rclcpp::spin_some(walk_node_); }

PyWalkWrapper::PyWalkWrapper(std::string ns, std::vector<py::bytes> parameter_msgs, bool force_smooth_step_transition) {
  // initialize rclcpp if not already done
  if (!rclcpp::contexts::get_global_default_context()->is_valid()) {
    rclcpp::init(0, nullptr);
  }

  // create parameters from serialized messages
  std::vector<rclcpp::Parameter> cpp_parameters = {};
  for (auto &parameter_msg : parameter_msgs) {
    cpp_parameters.push_back(
        rclcpp::Parameter::from_parameter_msg(fromPython<rcl_interfaces::msg::Parameter>(parameter_msg)));
  }
  walk_node_ = std::make_shared<bitbots_quintic_walk::WalkNode>(ns, cpp_parameters);
  set_robot_state(0);
  walk_node_->initializeEngine();
  walk_node_->getEngine()->setForceSmoothStepTransition(force_smooth_step_transition);
}

py::bytes PyWalkWrapper::step(double dt, py::bytes &cmdvel_msg, py::bytes &imu_msg, py::bytes &jointstate_msg,
                              py::bytes &pressure_left, py::bytes &pressure_right) {
  bitbots_msgs::msg::JointCommand result = walk_node_->step(
      dt, std::make_shared<geometry_msgs::msg::Twist>(fromPython<geometry_msgs::msg::Twist>(cmdvel_msg)),
      std::make_shared<sensor_msgs::msg::Imu>(fromPython<sensor_msgs::msg::Imu>(imu_msg)),
      std::make_shared<sensor_msgs::msg::JointState>(fromPython<sensor_msgs::msg::JointState>(jointstate_msg)),
      std::make_shared<bitbots_msgs::msg::FootPressure>(fromPython<bitbots_msgs::msg::FootPressure>(pressure_left)),
      std::make_shared<bitbots_msgs::msg::FootPressure>(fromPython<bitbots_msgs::msg::FootPressure>(pressure_right)));
  return toPython<bitbots_msgs::msg::JointCommand>(result);
}

py::bytes PyWalkWrapper::step_relative(double dt, py::bytes &step_msg, py::bytes &imu_msg, py::bytes &jointstate_msg,
                                       py::bytes &pressure_left, py::bytes &pressure_right) {
  bitbots_msgs::msg::JointCommand result = walk_node_->step_relative(
      dt, std::make_shared<geometry_msgs::msg::Twist>(fromPython<geometry_msgs::msg::Twist>(step_msg)),
      std::make_shared<sensor_msgs::msg::Imu>(fromPython<sensor_msgs::msg::Imu>(imu_msg)),
      std::make_shared<sensor_msgs::msg::JointState>(fromPython<sensor_msgs::msg::JointState>(jointstate_msg)),
      std::make_shared<bitbots_msgs::msg::FootPressure>(fromPython<bitbots_msgs::msg::FootPressure>(pressure_left)),
      std::make_shared<bitbots_msgs::msg::FootPressure>(fromPython<bitbots_msgs::msg::FootPressure>(pressure_right)));
  return toPython<bitbots_msgs::msg::JointCommand>(result);
}

py::bytes PyWalkWrapper::step_open_loop(double dt, py::bytes &cmdvel_msg) {
  geometry_msgs::msg::PoseArray result = walk_node_->step_open_loop(
      dt, std::make_shared<geometry_msgs::msg::Twist>(fromPython<geometry_msgs::msg::Twist>(cmdvel_msg)));
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

void PyWalkWrapper::reset() { walk_node_->reset(); }

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
  walk_node_->reset(walk_state, phase,
                    std::make_shared<geometry_msgs::msg::Twist>(fromPython<geometry_msgs::msg::Twist>(cmd_vel)),
                    reset_odometry);
}

double PyWalkWrapper::get_phase() { return walk_node_->getEngine()->getPhase(); }

double PyWalkWrapper::get_freq() { return walk_node_->getEngine()->getFreq(); }

py::bytes PyWalkWrapper::get_support_state() {
  biped_interfaces::msg::Phase support_state;
  if (walk_node_->getEngine()->isDoubleSupport()) {
    support_state.phase = biped_interfaces::msg::Phase::DOUBLE_STANCE;
  } else if (walk_node_->getEngine()->isLeftSupport()) {
    support_state.phase = biped_interfaces::msg::Phase::LEFT_STANCE;
  } else {
    support_state.phase = biped_interfaces::msg::Phase::RIGHT_STANCE;
  }
  return toPython<biped_interfaces::msg::Phase>(support_state);
}

bool PyWalkWrapper::is_left_support() { return walk_node_->getEngine()->isLeftSupport(); }

void PyWalkWrapper::set_robot_state(int state) {
  bitbots_msgs::msg::RobotControlState state_msg;
  state_msg.state = state;
  walk_node_->robotStateCb(std::make_shared<bitbots_msgs::msg::RobotControlState>(state_msg));
}

void PyWalkWrapper::set_parameter(py::bytes parameter_msg) {
  // convert serialized parameter msg to parameter object
  rclcpp::Parameter parameter =
      rclcpp::Parameter::from_parameter_msg(fromPython<rcl_interfaces::msg::Parameter>(parameter_msg));

  // needs to be a vector
  std::vector<rclcpp::Parameter> parameters = {parameter};
  walk_node_->onSetParameters(parameters);
}

void PyWalkWrapper::publish_debug() { walk_node_->publish_debug(); }

bool PyWalkWrapper::reset_and_test_if_speed_possible(py::bytes cmd_vel, double pos_threshold) {
  walk_node_->reset(bitbots_quintic_walk::WalkState::WALKING, 0.0,
                    std::make_shared<geometry_msgs::msg::Twist>(fromPython<geometry_msgs::msg::Twist>(cmd_vel)), true);
  bitbots_quintic_walk::WalkEngine *engine = walk_node_->getEngine();
  bitbots_quintic_walk::WalkIK *ik = walk_node_->getIk();
  bitbots_quintic_walk::WalkResponse current_response;
  bitbots_splines::JointGoals joint_goals;
  moveit::core::RobotStatePtr goal_state;
  goal_state.reset(new moveit::core::RobotState(*walk_node_->get_kinematic_model()));
  tf2::Vector3 support_off;
  tf2::Vector3 fly_off;
  tf2::Vector3 tf_vec_left;
  tf2::Vector3 tf_vec_right;
  Eigen::Vector3d l_transform;
  Eigen::Vector3d r_transform;
  int support_foot_changes = 0;
  bool last_support_foot = engine->isLeftSupport();

  while (support_foot_changes < 2) {
    current_response = engine->update(0.01);
    joint_goals = ik->calculate(current_response);

    // count finished half steps for stop condition
    if (engine->isLeftSupport() != last_support_foot) {
      last_support_foot = engine->isLeftSupport();
      support_foot_changes++;
    }

    tf2::Transform trunk_to_support_foot = current_response.support_foot_to_trunk.inverse();
    tf2::Transform trunk_to_flying_foot = trunk_to_support_foot * current_response.support_foot_to_flying_foot;

    // set joints in the state to compute forward kinematics
    std::vector<std::string> names = joint_goals.first;
    std::vector<double> goals = joint_goals.second;
    for (size_t i = 0; i < names.size(); i++) {
      // besides its name, this method only changes a single joint position...
      goal_state->setJointPositions(names[i], &goals[i]);
    }
    goal_state->updateLinkTransforms();

    // read out forward kinematics and compare
    l_transform = goal_state->getGlobalLinkTransform("l_sole").translation();
    r_transform = goal_state->getGlobalLinkTransform("r_sole").translation();
    tf2::convert(l_transform, tf_vec_left);
    tf2::convert(r_transform, tf_vec_right);
    if (current_response.is_left_support_foot) {
      support_off = trunk_to_support_foot.getOrigin() - tf_vec_left;
      fly_off = trunk_to_flying_foot.getOrigin() - tf_vec_right;
    } else {
      support_off = trunk_to_support_foot.getOrigin() - tf_vec_right;
      fly_off = trunk_to_flying_foot.getOrigin() - tf_vec_left;
    }

    double pos_offset = std::abs(support_off.x()) + std::abs(support_off.y()) + std::abs(support_off.z()) +
                        std::abs(fly_off.x()) + std::abs(fly_off.y()) + std::abs(fly_off.z());
    // RCLCPP_WARN(walk_node_->get_logger(), "%f", pos_offset);
    //  todo orientation offset not so simple due to quaternions but could be checked too. typically the position is
    //  enough to see that the IK solution is wrong
    if (pos_offset > pos_threshold) {
      return false;
    }
  }
  return true;
}

PYBIND11_MODULE(libpy_quintic_walk, m) {
  using namespace bitbots_quintic_walk;

  py::class_<PyWalkWrapper, std::shared_ptr<PyWalkWrapper>>(m, "PyWalkWrapper")
      .def(py::init<std::string, std::vector<py::bytes>, bool>())
      .def("step", &PyWalkWrapper::step)
      .def("step_relative", &PyWalkWrapper::step_relative)
      .def("step_open_loop", &PyWalkWrapper::step_open_loop)
      .def("get_left_foot_pose", &PyWalkWrapper::get_left_foot_pose)
      .def("get_right_foot_pose", &PyWalkWrapper::get_right_foot_pose)
      .def("set_robot_state", &PyWalkWrapper::set_robot_state)
      .def("reset", &PyWalkWrapper::reset)
      .def("special_reset", &PyWalkWrapper::special_reset)
      .def("set_parameter", &PyWalkWrapper::set_parameter)
      .def("get_phase", &PyWalkWrapper::get_phase)
      .def("get_freq", &PyWalkWrapper::get_freq)
      .def("get_odom", &PyWalkWrapper::get_odom)
      .def("spin_some", &PyWalkWrapper::spin_some)
      .def("publish_debug", &PyWalkWrapper::publish_debug)
      .def("get_support_state", &PyWalkWrapper::get_support_state)
      .def("is_left_support", &PyWalkWrapper::is_left_support)
      .def("reset_and_test_if_speed_possible", &PyWalkWrapper::reset_and_test_if_speed_possible);
}
