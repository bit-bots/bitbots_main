#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/convert.h>
#include <pybind11/pybind11.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
namespace py = pybind11;


/**
 * Convert the result of a ROS message serialized in Python to a C++ message
 * @tparam T C++ ROS message type
 * @param bytes message serialized in Python
 * @return the converted message
 */
template <typename T>
T from_python(py::bytes &bytes) {
  // buffer_info is used to extract data pointer and size of bytes
  py::buffer_info info(py::buffer(bytes).request());

  // initialize serialized message struct
  rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();
  auto allocator = rcl_get_default_allocator();
  rmw_serialized_message_init(&serialized_message, info.size, &allocator);
  serialized_message.buffer = reinterpret_cast<uint8_t *>(info.ptr);
  serialized_message.buffer_length = info.size;
  auto ts = rosidl_typesupport_cpp::get_message_type_support_handle<T>();

  T out;
  // do the deserialization
  rmw_ret_t result2 = rmw_deserialize(&serialized_message, ts, &out);
  if (result2 != RMW_RET_OK) {
    printf("Failed to deserialize message!\n");
  }

  return out;
}

/**
 * Convert a C++ ROS message to a Python ByteString that can be deserialized to the message
 * @tparam T C++ ROS message type
 * @param msg C++ ROS message
 * @return the message serialized into a Python ByteString
 */
template <typename T>
py::bytes to_python(T& msg) {
  // initialize serialized message struct
  rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();
  auto type_support = rosidl_typesupport_cpp::get_message_type_support_handle<T>();
  auto allocator = rcl_get_default_allocator();
  rmw_serialized_message_init(&serialized_message, 0u, &allocator);

  // do the serialization
  rmw_ret_t result = rmw_serialize(&msg, type_support, &serialized_message);
  if (result != RMW_RET_OK) {
    printf("Failed to serialize message!\n");
  }

  // convert the result to python bytes by using the data and length
  return {reinterpret_cast<const char *>(serialized_message.buffer), serialized_message.buffer_length};
}

class BitbotsMoveitBindings : public rclcpp::Node
{
 public:
  BitbotsMoveitBindings():Node("BitbotsMoveitBindings"){
    /*std::string robot_description = "robot_description";
    robot_model_loader::RobotModelLoader loader(SharedPtr(this), robot_description, false);
    robot_model_ = loader.getModel();
    if (!robot_model_) {
      RCLCPP_ERROR(this->get_logger(), "failed to load robot model %s", robot_description.c_str());
    }*/
  }
/*
  planning_scene::PlanningSceneSharedPtr getPlanningScene() {
    std::string robot_description = "robot_description";
    static auto planning_scene_monitor =
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_description);
    planning_scene::PlanningSceneSharedPtr planning_scene =
        planning_scene_monitor->getPlanningScene();
    if (!planning_scene) {
      RCLCPP_ERROR_ONCE(this->get_logger(), "failed to connect to planning scene");
    }
    return planning_scene;
  }

  py::bytes getPositionIK(const std::string& request_str, bool approximate = false) {
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
  */
  py::bytes getPositionFK(py::bytes &msg) {
    auto request = from_python<moveit_msgs::srv::GetPositionFK::Request>(msg);

    moveit_msgs::srv::GetPositionFK::Response response;
    if (!robot_model_) {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_OBJECT_NAME;
      return to_python(response);
    }
    return to_python(response);
  }
  /*
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
  */
 private:
  moveit::core::RobotModelPtr robot_model_;
};

void initRos() {
  //char *argv[] = {NULL};
  rclcpp::init(0, nullptr);
}

PYBIND11_MODULE(libbitbots_moveit_bindings, m) {
  m.def("initRos", &initRos);
  py::class_<BitbotsMoveitBindings, std::shared_ptr<BitbotsMoveitBindings>>(m, "BitbotsMoveitBindings")
  .def(py::init<>())
  //.def("getPositionIK", &getPositionIK, "Calls the IK to provide a solution")
  .def("getPositionFK", &BitbotsMoveitBindings::getPositionFK);
}