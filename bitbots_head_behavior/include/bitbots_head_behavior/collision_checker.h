#include <Python.h>
#include <boost/python.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <iostream>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/msg/joint_state.hpp>

class CollisionChecker {
 public:
  CollisionChecker();
  void set_head_motors(double pan, double tilt);
  void set_joint_states(std::string msg);
  bool check_collision();

 private:
  planning_scene::PlanningScenePtr planning_scene_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_;
  moveit::core::msg::RobotStatePtr robot_state_;
};
