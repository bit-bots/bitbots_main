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
#include <rclcpp/serialization.hpp>
namespace py = pybind11;

typedef void destroy_ros_message_function (void *);

std::unique_ptr<void, destroy_ros_message_function *> create_from_py(py::object pymessage) {
  typedef void * create_ros_message_function (void);

  py::object pymetaclass = pymessage.attr("__class__");

  py::object value = pymetaclass.attr("_CREATE_ROS_MESSAGE");
  auto capsule_ptr = static_cast<void *>(value.cast<py::capsule>());
  auto create_ros_message =
      reinterpret_cast<create_ros_message_function *>(capsule_ptr);
  if (!create_ros_message) {
    throw py::error_already_set();
  }

  value = pymetaclass.attr("_DESTROY_ROS_MESSAGE");
  capsule_ptr = static_cast<void *>(value.cast<py::capsule>());
  auto destroy_ros_message =
      reinterpret_cast<destroy_ros_message_function *>(capsule_ptr);
  if (!destroy_ros_message) {
    throw py::error_already_set();
  }

  void * message = create_ros_message();
  if (!message) {
    throw std::bad_alloc();
  }
  return std::unique_ptr<
      void, destroy_ros_message_function *>(message, destroy_ros_message);
}

std::unique_ptr<void, destroy_ros_message_function *> convert_from_py(py::object pymessage) {
  typedef bool convert_from_py_signature (PyObject *, void *);

  std::unique_ptr<void, destroy_ros_message_function *> message =
      create_from_py(pymessage);

  py::object pymetaclass = pymessage.attr("__class__");

  auto capsule_ptr = static_cast<void *>(
      pymetaclass.attr("_CONVERT_FROM_PY").cast<py::capsule>());
  auto convert =
      reinterpret_cast<convert_from_py_signature *>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }

  if (!convert(pymessage.ptr(), message.get())) {
    throw py::error_already_set();
  }

  return message;
}

py::object convert_to_py(void * message, py::object pyclass) {
  py::object pymetaclass = pyclass.attr("__class__");

  auto capsule_ptr = static_cast<void *>(
      pymetaclass.attr("_CONVERT_TO_PY").cast<py::capsule>());

  typedef PyObject * convert_to_py_function (void *);
  auto convert = reinterpret_cast<convert_to_py_function *>(capsule_ptr);
  if (!convert) {
    throw py::error_already_set();
  }
  return py::reinterpret_steal<py::object>(convert(message));
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
  virtual ~BitbotsMoveitBindings() = default;
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
  py::bytes getPositionFK(const py::object& request) {
    auto ros_msg = convert_from_py(request);

    rclcpp::Serialization<moveit_msgs::srv::GetPositionFK::Request> serializer_request;
    rclcpp::Serialization<moveit_msgs::srv::GetPositionFK::Response> serializer_response;
    // deserialize request from string to msg
    printf("%s\n", ((moveit_msgs::srv::GetPositionFK::Request *) ros_msg.get())->header.frame_id.c_str());

    /*moveit_msgs::srv::GetPositionFK::Response response;
    if (!robot_model_) {
      response.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_OBJECT_NAME;
      rclcpp::SerializedMessage response_serialized;
      serializer_response.serialize_message(response, &response_serialized);
      auto response_str = response_serialized.get_rcl_serialized_message();
      return response_str.buffer;
    }*/
    return {};
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