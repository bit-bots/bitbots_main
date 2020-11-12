#include <bitbots_head_behavior/collision_checker.h>

CollisionChecker::CollisionChecker() {
  std::map<std::string, std::string> empty;
  ros::init(empty, "collision_checker", ros::init_options::AnonymousName);
  robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("/robot_description", false));
  robot_model_ = robot_model_loader_->getModel();
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
}

void CollisionChecker::set_head_motors(double pan, double tilt) {
  robot_state_->setJointPositions("HeadPan", &pan);
  robot_state_->setJointPositions("HeadTilt", &tilt);
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
        .def("check_collision", &CollisionChecker::check_collision, "Returns true if the head collides, else false");
    }
