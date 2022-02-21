#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include <pybind11/pybind11.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <ros2_python_extension/init.hpp>
#include <ros2_python_extension/serialization.hpp>
#include "rcl_interfaces/srv/get_parameters.hpp"
namespace py = pybind11;
using namespace std::chrono_literals;

class BitbotsMoveitBindings {
 public:
  BitbotsMoveitBindings() {
    rclcpp::NodeOptions options = rclcpp::NodeOptions().allow_undeclared_parameters(true);
    node_ = std::make_shared<rclcpp::Node>("BitbotsMoveitBindings", options);

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_, "/move_group");
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    }
    rcl_interfaces::msg::ListParametersResult parameter_list = parameters_client->list_parameters({"robot_description_kinematics"},10);
    /*for (auto & name : parameter_list.names) {
      RCLCPP_INFO(node_->get_logger(), "Found parameter %s", name.c_str());
    }*/
    auto parameters = parameters_client->get_parameters(parameter_list.names);

    std::stringstream ss;
    // Get a few of the parameters just set.
    /*for (auto & parameter : parameters)
    {
      RCLCPP_INFO(node_->get_logger(), "Parameter name: %s ", parameter.get_name().c_str());
      RCLCPP_INFO(node_->get_logger(), "Parameter value %s", parameter.value_to_string().c_str());
    }*/
    node_->set_parameters(parameters);

    std::string robot_description = "robot_description";
    // get the robot description from the blackboard
    robot_model_loader::RobotModelLoader loader(node_, robot_description, false);
    robot_model_ = loader.getModel();
    if (!robot_model_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "failed to load robot model %s. Did you start the blackboard (bitbots_bringup load_robot_description.launch)?",
                   robot_description.c_str());
    }
    robot_state_.reset(new moveit::core::RobotState(robot_model_));

    auto planning_scene_monitor =
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, robot_description);
    planning_scene_ = planning_scene_monitor->getPlanningScene();
    if (!planning_scene_) {
      RCLCPP_ERROR_ONCE(node_->get_logger(), "failed to connect to planning scene");
    }

    /*while(rclcpp::ok()){
      rclcpp::spin(node_);
    }*/
  }

  py::bytes getPositionIK(py::bytes &msg, bool approximate = false) {
    auto request = ros2_python_extension::fromPython<moveit_msgs::srv::GetPositionIK::Request>(msg);
    moveit_msgs::srv::GetPositionIK::Response response;
    if (!robot_model_) {
      response.error_code.val =
          moveit_msgs::msg::MoveItErrorCodes::INVALID_OBJECT_NAME;
      return ros2_python_extension::toPython<moveit_msgs::srv::GetPositionIK::Response>(response);
    }

    auto joint_model_group =
        robot_model_->getJointModelGroup(request.ik_request.group_name);
    if (!joint_model_group) {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return ros2_python_extension::toPython<moveit_msgs::srv::GetPositionIK::Response>(response);
    }

    robot_state_->update();
    if (!request.ik_request.robot_state.joint_state.name.empty() ||
        !request.ik_request.robot_state.multi_dof_joint_state.joint_names
            .empty()) {
      moveit::core::robotStateMsgToRobotState(request.ik_request.robot_state, *robot_state_);
      robot_state_->update();
    }

    bool success = true;

    kinematics::KinematicsQueryOptions ik_options;
    ik_options.return_approximate_solution = approximate;

    moveit::core::GroupStateValidityCallbackFn callback;
    if (request.ik_request.avoid_collisions) {
      callback = [this](moveit::core::RobotState *state,
                        const moveit::core::JointModelGroup *group,
                        const double *values) {
        state->setJointGroupPositions(group, values);
        state->update();
        return !planning_scene_ || !planning_scene_->isStateColliding(*state, group->getName());
      };
    }

    if (request.ik_request.pose_stamped_vector.empty()) {
      if (request.ik_request.ik_link_name.empty()) {
        success = robot_state_->setFromIK(
            joint_model_group, request.ik_request.pose_stamped.pose,
            rclcpp::Duration(request.ik_request.timeout).seconds(),
            callback, ik_options);
      } else {
        success = robot_state_->setFromIK(
            joint_model_group, request.ik_request.pose_stamped.pose,
            request.ik_request.ik_link_name, rclcpp::Duration(request.ik_request.timeout).seconds(),
            callback, ik_options);
      }
    } else {
      EigenSTL::vector_Isometry3d poses;
      poses.reserve(request.ik_request.pose_stamped_vector.size());
      for (auto &pose: request.ik_request.pose_stamped_vector) {
        poses.emplace_back();
        tf2::convert(pose.pose, poses.back());
      }
      if (request.ik_request.ik_link_names.empty()) {
        std::vector<std::string> end_effector_names;
        joint_model_group->getEndEffectorTips(end_effector_names);
        success = robot_state_->setFromIK(
            joint_model_group, poses, end_effector_names,
            rclcpp::Duration(request.ik_request.timeout).seconds(),
            callback, ik_options);
      } else {
        success = robot_state_->setFromIK(
            joint_model_group, poses, request.ik_request.ik_link_names,
            rclcpp::Duration(request.ik_request.timeout).seconds(),
            callback, ik_options);
      }
    }

    robot_state_->update();

    moveit::core::robotStateToRobotStateMsg(*robot_state_, response.solution);
    if (success) {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    } else {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    }
    return ros2_python_extension::toPython<moveit_msgs::srv::GetPositionIK::Response>(response);
  }

  py::bytes getPositionFK(py::bytes &msg) {
    auto request = ros2_python_extension::fromPython<moveit_msgs::srv::GetPositionFK::Request>(msg);

    moveit_msgs::srv::GetPositionFK::Response response;
    if (!robot_model_) {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_OBJECT_NAME;
      return ros2_python_extension::toPython(response);
    }

    sensor_msgs::msg::JointState joint_state = request.robot_state.joint_state;
    for (size_t i = 0; i < joint_state.name.size(); ++i) {
      robot_state_->setJointPositions(joint_state.name[i], &joint_state.position[i]);
    }
    robot_state_->update();
    Eigen::Isometry3d pose;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = request.header;
    for (std::string &link_name: request.fk_link_names) {
      if (!robot_model_->hasLinkModel(link_name)) {
        response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_LINK_NAME;
        return ros2_python_extension::toPython<moveit_msgs::srv::GetPositionFK::Response>(response);
      }
      response.fk_link_names.push_back(link_name);
      pose = robot_state_->getGlobalLinkTransform(link_name);
      pose_stamped.pose = tf2::toMsg(pose);
      response.pose_stamped.push_back(pose_stamped);
    }
    response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return ros2_python_extension::toPython<moveit_msgs::srv::GetPositionFK::Response>(response);
  }

 private:
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  planning_scene::PlanningScenePtr planning_scene_;
  std::shared_ptr<rclcpp::Node> node_;
};

PYBIND11_MODULE(libbitbots_moveit_bindings, m) {
  m.def("initRos", &ros2_python_extension::initRos);
  py::class_<BitbotsMoveitBindings, std::shared_ptr<BitbotsMoveitBindings>>(m, "BitbotsMoveitBindings")
      .def(py::init<>())
      .def("getPositionIK", &BitbotsMoveitBindings::getPositionIK)
      .def("getPositionFK", &BitbotsMoveitBindings::getPositionFK);
}
