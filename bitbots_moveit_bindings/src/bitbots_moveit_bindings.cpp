#include <Python.h>
#include <boost/python.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_position_ik.h>
#include <moveit_msgs/srv/get_position_fk.h>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include <pybind11/pybind11.h>

/**
 * Read a ROS message from a serialized string.
 */
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

/**
 * Write a ROS message into a serialized string.
 */
template<typename M>
std::string to_python(const M &msg) {
  size_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i) {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}

moveit::core::RobotModelPtr getRobotModel() {
  std::string robot_description = "robot_description";
  static robot_model_loader::RobotModelLoader loader(robot_description);
  moveit::core::RobotModelPtr robot_model = loader.getModel();
  if (!robot_model) {
    RCLCPP_ERROR(this->get_logger(),"failed to load robot model %s", robot_description.c_str());
  }
  return robot_model;
}

planning_scene::PlanningSceneSharedPtr getPlanningScene() {
  std::string robot_description = "robot_description";
  static auto planning_scene_monitor =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description);
  planning_scene::PlanningSceneSharedPtr planning_scene =
      planning_scene_monitor->getPlanningScene();
  if (!planning_scene) {
    RCLCPP_ERROR_ONCE(this->get_logger(),"failed to connect to planning scene");
  }
  return planning_scene;
}

moveit::py_bindings_tools::ByteString getPositionIK(const std::string& request_str, bool approximate = false) {
  auto request = from_python<moveit_msgs::GetPositionIK::Request>(request_str);
  moveit_msgs::GetPositionIK::Response response;
  static moveit::core::RobotModelPtr robot_model = getRobotModel();
  if (!robot_model) {
    response.error_code.val =
        moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME;
    return moveit::py_bindings_tools::serializeMsg(to_python<moveit_msgs::GetPositionIK::Response>(response));
  }

  auto joint_model_group =
      robot_model->getJointModelGroup(request.ik_request.group_name);
  if (!joint_model_group) {
    response.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GROUP_NAME;
    return moveit::py_bindings_tools::serializeMsg(to_python<moveit_msgs::GetPositionIK::Response>(response));
  }

  static moveit::core::msg::RobotState robot_state(robot_model);
  robot_state.update();

  if (!request.ik_request.robot_state.joint_state.name.empty() ||
      !request.ik_request.robot_state.multi_dof_joint_state.joint_names
           .empty()) {
    moveit::core::robotStateMsgToRobotState(request.ik_request.robot_state,
                                            robot_state);
    robot_state.update();
  }

  bool success = true;

  kinematics::KinematicsQueryOptions ik_options;
  ik_options.return_approximate_solution = approximate;

  moveit::core::GroupStateValidityCallbackFn callback;
  if (request.ik_request.avoid_collisions) {
    callback = [](moveit::core::msg::RobotState *state,
                  const moveit::core::JointModelGroup *group,
                  const double *values) {
      auto planning_scene = getPlanningScene();
      state->setJointGroupPositions(group, values);
      state->update();
      return !planning_scene ||
             !planning_scene->isStateColliding(*state, group->getName());
    };
  }

  if (request.ik_request.pose_stamped_vector.empty()) {
    if (request.ik_request.ik_link_name.empty()) {
      success = robot_state.setFromIK(
          joint_model_group, request.ik_request.pose_stamped.pose,
          request.ik_request.timeout.toSec(),
          callback, ik_options);
    } else {
      success = robot_state.setFromIK(
          joint_model_group, request.ik_request.pose_stamped.pose,
          request.ik_request.ik_link_name, request.ik_request.timeout.toSec(),
          callback, ik_options);
    }
  } else {
    EigenSTL::vector_Isometry3d poses;
    poses.reserve(request.ik_request.pose_stamped_vector.size());
    for (auto &pose : request.ik_request.pose_stamped_vector) {
      poses.emplace_back();
      tf2::convert(pose.pose, poses.back());
    }
    if (request.ik_request.ik_link_names.empty()) {
      std::vector<std::string> end_effector_names;
      joint_model_group->getEndEffectorTips(end_effector_names);
      success = robot_state.setFromIK(
          joint_model_group, poses, end_effector_names,
          request.ik_request.timeout.toSec(),
          callback, ik_options);
    } else {
      success = robot_state.setFromIK(
          joint_model_group, poses, request.ik_request.ik_link_names,
          request.ik_request.timeout.toSec(),
          callback, ik_options);
    }
  }

  robot_state.update();

  moveit::core::robotStateToRobotStateMsg(robot_state, response.solution);
  if (success) {
    response.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  } else {
    response.error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  }
  return moveit::py_bindings_tools::serializeMsg(to_python<moveit_msgs::GetPositionIK::Response>(response));
}

moveit::py_bindings_tools::ByteString getPositionFK(const std::string& request_str) {
  auto request = from_python<moveit_msgs::GetPositionFK::Request>(request_str);
  moveit_msgs::GetPositionFK::Response response;
  
  static moveit::core::RobotModelPtr robot_model = getRobotModel();
  if (!robot_model) {
    response.error_code.val =
        moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME;
    return moveit::py_bindings_tools::serializeMsg(to_python<moveit_msgs::GetPositionFK::Response>(response));
  }

  static moveit::core::msg::RobotState robot_state(robot_model);
  sensor_msgs::msg::JointState joint_state = request.robot_state.joint_state;
  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    robot_state.setJointPositions(joint_state.name[i], &joint_state.position[i]);
  }
  robot_state.update();
  Eigen::Isometry3d pose;
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = request.header;
  for (std::string& link_name : request.fk_link_names) {
    if (!robot_model->hasLinkModel(link_name)) {
      response.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_LINK_NAME;
      return moveit::py_bindings_tools::serializeMsg(to_python<moveit_msgs::GetPositionFK::Response>(response));
    }
    response.fk_link_names.push_back(link_name);
    pose = robot_state.getGlobalLinkTransform(link_name);
    pose_stamped.pose = tf2::toMsg(pose);
    response.pose_stamped.push_back(pose_stamped);
  }
  response.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return moveit::py_bindings_tools::serializeMsg(to_python<moveit_msgs::GetPositionFK::Response>(response));
}

PYBIND11_MODULE(bitbots_moveit_bindings, m)
{
    m.def("getPositionIK", &getPositionIK, "Calls the IK to provide a solution");
    m.def("getPositionFK", &getPositionFK, "Calls the FK to provide a solution");
}