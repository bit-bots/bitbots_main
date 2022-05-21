#include <Python.h>
#include <boost/python.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <map>
#include <iostream>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <ros2_python_extension/serialization.hpp>

using namespace ros2_python_extension;

python_parameters(std::vector<py::bytes> parameter_msgs);

class CollisionChecker : public rclcpp::Node {
 public:
  CollisionChecker(std::vector<py::bytes> parameter_msgs);
  void set_head_motors(double pan, double tilt);
  void set_joint_states(py::bytes msg);
  bool check_collision();

 private:
  planning_scene::PlanningScenePtr planning_scene_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
};
