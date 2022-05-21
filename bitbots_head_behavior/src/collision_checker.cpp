#include <bitbots_head_behavior/collision_checker.h>

python_parameters(std::vector<py::bytes> parameter_msgs) {
  // create parameters from serialized messages
  std::vector<rclcpp::Parameter> cpp_parameters = {};
  for (auto &parameter_msg: parameter_msgs) {
    cpp_parameters
        .push_back(rclcpp::Parameter::from_parameter_msg(fromPython<rcl_interfaces::msg::Parameter>(parameter_msg)));
  }
  return cpp_parameters
}

CollisionChecker::CollisionChecker(std::vector<py::bytes> parameter_msgs):
    Node("collision_checker", rclcpp::NodeOptions().allow_undeclared_parameters(true).parameter_overrides(python_parameters(parameters)).automatically_declare_parameters_from_overrides(true)){
  //load MoveIt! model
  robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(SharedPtr(this), "robot_description", true);
  robot_model_ = robot_model_loader_->getModel();
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  robot_state_.reset(new moveit::core::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
}

void CollisionChecker::set_head_motors(double pan, double tilt) {
  robot_state_->setJointPositions("HeadPan", &pan);
  robot_state_->setJointPositions("HeadTilt", &tilt);
}

void CollisionChecker::set_joint_states(py::bytes msg) {
  sensor_msgs::msg::JointState joint_states = fromPython<sensor_msgs::msg::JointState>(msg);
  for (size_t i = 0; i < joint_states.name.size(); ++i) {
    robot_state_->setJointPositions(joint_states.name[i], &joint_states.position[i]);
  }
}

bool CollisionChecker::check_collision() {
  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
  planning_scene_->checkCollision(req, res, *robot_state_, acm);
  return res.collision;
}



PYBIND11_MODULE(collision_checker, m){
    py::class_<CollisionChecker, std::shared_ptr<CollisionChecker>>(m, "CollisionChecker")
    .def(py::init<std::vector<py::bytes>>()>())
    .def("set_head_motors", &CollisionChecker::set_head_motors, "Set the current pan and tilt joint values [radian]", py::arg("pan"), py::arg("tilt"))
    .def("set_joint_states", &CollisionChecker::set_joint_states, "Set the current joint states")
    .def("check_collision", &CollisionChecker::check_collision, "Returns true if the head collides, else false");
}
