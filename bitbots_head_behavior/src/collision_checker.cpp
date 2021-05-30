#include <bitbots_head_behavior/collision_checker.h>

/* Read a ROS message from a serialized string. */
template<typename M>
M from_python(const std::string &str_msg) {
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i) {
    buffer[i] = str_msg[i];
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  M msg;
  ros::serialization::Serializer<M>::read(stream, msg);
  return msg;
}

CollisionChecker::CollisionChecker() {
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description", false));
  robot_model_ = robot_model_loader_->getModel();
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
  ROS_INFO("Collision checker setup finished");
}

void CollisionChecker::set_head_motors(double pan, double tilt) {
  robot_state_->setJointPositions("HeadPan", &pan);
  robot_state_->setJointPositions("HeadTilt", &tilt);
}

void CollisionChecker::set_joint_states(std::string msg) {
  sensor_msgs::JointState joint_states = from_python<sensor_msgs::JointState>(msg);
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

BOOST_PYTHON_MODULE(collision_checker)
    {
        using namespace boost::python;

        class_<CollisionChecker>("CollisionChecker", init<>())
        .def("set_head_motors", &CollisionChecker::set_head_motors, (arg("pan"), arg("tilt")), "Set the current pan and tilt moter values [radian]")
        .def("set_joint_states", &CollisionChecker::set_joint_states, "Set the current joint states")
        .def("check_collision", &CollisionChecker::check_collision, "Returns true if the head collides, else false");
    }
