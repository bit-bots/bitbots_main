#include <bio_ik/bio_ik.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <tf2/convert.h>

#include <bio_ik_msgs/msg/ik_request.hpp>
#include <bio_ik_msgs/msg/ik_response.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_python_extension/serialization.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>

#include "rcl_interfaces/srv/get_parameters.hpp"
namespace py = pybind11;
using namespace std::chrono_literals;
using std::placeholders::_1;

class BitbotsMoveitBindings {
public:
  BitbotsMoveitBindings(std::string node_name, std::vector<py::bytes> parameter_msgs = {}) {
    // initialize rclcpp if not already done
    if (!rclcpp::contexts::get_global_default_context()->is_valid()) {
      rclcpp::init(0, nullptr);
    }

    // deserialize parameters
    std::vector<rclcpp::Parameter> cpp_parameters = {};
    for (auto& parameter_msg : parameter_msgs) {
      cpp_parameters.push_back(rclcpp::Parameter::from_parameter_msg(
          ros2_python_extension::fromPython<rcl_interfaces::msg::Parameter>(parameter_msg)));
    }

    rclcpp::NodeOptions options = rclcpp::NodeOptions()
                                      .allow_undeclared_parameters(true)
                                      .parameter_overrides(cpp_parameters)
                                      .automatically_declare_parameters_from_overrides(true);
    node_ = std::make_shared<rclcpp::Node>(node_name, options);
    // set logging level to warn to reduce spam
    node_->get_logger().set_level(rclcpp::Logger::Level::Warn);

    if (cpp_parameters.empty()) {
      // get all kinematics parameters from the move_group node if they are not
      // provided via the constructor
      auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_, "/move_group");
      while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
          rclcpp::shutdown();
        }
        RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
      }
      rcl_interfaces::msg::ListParametersResult parameter_list =
          parameters_client->list_parameters({"robot_description_kinematics"}, 10);
      auto copied_parameters = parameters_client->get_parameters(parameter_list.names);

      // set the parameters to our node
      node_->set_parameters(copied_parameters);
    }
    // now that all parameters are set we can load the robot model and the
    // kinematic solvers
    std::string robot_description = "robot_description";
    // get the robot description from the blackboard
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
        robot_model_loader::RobotModelLoader(node_, robot_description, true));
    robot_model_ = loader_->getModel();
    if (!robot_model_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "failed to load robot model %s. Did you start the "
                   "blackboard (bitbots_bringup base.launch)?",
                   robot_description.c_str());
    }
    robot_state_.reset(new moveit::core::RobotState(robot_model_));
    robot_state_->setToDefaultValues();

    // get planning scene for collision checking
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, robot_description);
    planning_scene_ = planning_scene_monitor_->getPlanningScene();
    if (!planning_scene_) {
      RCLCPP_ERROR_ONCE(node_->get_logger(), "failed to connect to planning scene");
    }
    exec_ = std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
    exec_->add_node(node_);
  }

  py::bytes getPositionIK(py::bytes& msg, bool approximate = false) {
    auto request = ros2_python_extension::fromPython<moveit_msgs::srv::GetPositionIK::Request>(msg);
    moveit_msgs::srv::GetPositionIK::Response response;
    if (!robot_model_) {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_OBJECT_NAME;
      return ros2_python_extension::toPython<moveit_msgs::srv::GetPositionIK::Response>(response);
    }

    auto joint_model_group = robot_model_->getJointModelGroup(request.ik_request.group_name);
    if (!joint_model_group) {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      return ros2_python_extension::toPython<moveit_msgs::srv::GetPositionIK::Response>(response);
    }

    robot_state_->update();
    if (!request.ik_request.robot_state.joint_state.name.empty() ||
        !request.ik_request.robot_state.multi_dof_joint_state.joint_names.empty()) {
      moveit::core::robotStateMsgToRobotState(request.ik_request.robot_state, *robot_state_);
      robot_state_->update();
    }

    bool success = true;

    kinematics::KinematicsQueryOptions ik_options;
    ik_options.return_approximate_solution = approximate;

    moveit::core::GroupStateValidityCallbackFn callback;
    if (request.ik_request.avoid_collisions) {
      callback = [this](moveit::core::RobotState* state, const moveit::core::JointModelGroup* group,
                        const double* values) {
        state->setJointGroupPositions(group, values);
        state->update();
        return !planning_scene_ || !planning_scene_->isStateColliding(*state, group->getName());
      };
    }

    if (request.ik_request.pose_stamped_vector.empty()) {
      if (request.ik_request.ik_link_name.empty()) {
        success = robot_state_->setFromIK(joint_model_group, request.ik_request.pose_stamped.pose,
                                          rclcpp::Duration(request.ik_request.timeout).seconds(), callback, ik_options);
      } else {
        success = robot_state_->setFromIK(joint_model_group, request.ik_request.pose_stamped.pose,
                                          request.ik_request.ik_link_name,
                                          rclcpp::Duration(request.ik_request.timeout).seconds(), callback, ik_options);
      }
    } else {
      EigenSTL::vector_Isometry3d poses;
      poses.reserve(request.ik_request.pose_stamped_vector.size());
      for (auto& pose : request.ik_request.pose_stamped_vector) {
        poses.emplace_back();
        tf2::convert(pose.pose, poses.back());
      }
      if (request.ik_request.ik_link_names.empty()) {
        std::vector<std::string> end_effector_names;
        joint_model_group->getEndEffectorTips(end_effector_names);
        success = robot_state_->setFromIK(joint_model_group, poses, end_effector_names,
                                          rclcpp::Duration(request.ik_request.timeout).seconds(), callback, ik_options);
      } else {
        success = robot_state_->setFromIK(joint_model_group, poses, request.ik_request.ik_link_names,
                                          rclcpp::Duration(request.ik_request.timeout).seconds(), callback, ik_options);
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

  py::bytes getPositionFK(py::bytes& msg) {
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
    for (std::string& link_name : request.fk_link_names) {
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

  py::bytes getBioIKIK(py::bytes& msg) {
    // extra method to use BioIK specific goals
    auto request = ros2_python_extension::fromPython<bio_ik_msgs::msg::IKRequest>(msg);

    bio_ik::BioIKKinematicsQueryOptions ik_options;
    ik_options.return_approximate_solution = request.approximate;
    ik_options.fixed_joints = request.fixed_joints;
    ik_options.replace = true;

    convertGoals(request, ik_options);

    moveit::core::GroupStateValidityCallbackFn callback;
    if (request.avoid_collisions) {
      std::cout << "Avoid collisions not implemented in bitbots_moveit_bindings";
      exit(1);
    }
    auto joint_model_group = robot_model_->getJointModelGroup(request.group_name);
    if (!joint_model_group) {
      std::cout << "Group name in IK call not specified";
      exit(1);
    }

    float timeout_seconds = request.timeout.sec + request.timeout.nanosec * 1e9;
    bool success = robot_state_->setFromIK(joint_model_group, EigenSTL::vector_Isometry3d(), std::vector<std::string>(),
                                           timeout_seconds, callback, ik_options);

    robot_state_->update();

    bio_ik_msgs::msg::IKResponse response;
    moveit::core::robotStateToRobotStateMsg(*robot_state_, response.solution);
    response.solution_fitness = ik_options.solution_fitness;
    if (success) {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    } else {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    }
    return ros2_python_extension::toPython<bio_ik_msgs::msg::IKResponse>(response);
  }

  void setHeadMotors(double pan, double tilt) {
    robot_state_->setJointPositions("HeadPan", &pan);
    robot_state_->setJointPositions("HeadTilt", &tilt);
  }

  void setJointStates(py::bytes msg) {
    sensor_msgs::msg::JointState joint_states = ros2_python_extension::fromPython<sensor_msgs::msg::JointState>(msg);
    for (size_t i = 0; i < joint_states.name.size(); ++i) {
      robot_state_->setJointPositions(joint_states.name[i], &joint_states.position[i]);
    }
  }

  bool checkCollision() {
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
    planning_scene_->checkCollision(req, res, *robot_state_, acm);
    return res.collision;
  }

  py::bytes getJointStates() {
    sensor_msgs::msg::JointState joint_state;
    robotStateToJointStateMsg(*robot_state_, joint_state);
    return ros2_python_extension::toPython<sensor_msgs::msg::JointState>(joint_state);
  }

private:
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_scene::PlanningScenePtr planning_scene_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::experimental::executors::EventsExecutor> exec_;
  std::thread t_;

  static tf2::Vector3 p(const geometry_msgs::msg::Point& p) {
    return tf2::Vector3(p.x, p.y, p.z);
  }

  static tf2::Vector3 p(const geometry_msgs::msg::Vector3& p) {
    return tf2::Vector3(p.x, p.y, p.z);
  }

  static tf2::Quaternion q(const geometry_msgs::msg::Quaternion& q) {
    return tf2::Quaternion(q.x, q.y, q.z, q.w);
  }

  static double w(double w, double def = 1.0) {
    if (w == 0 || !std::isfinite(w))
      w = def;
    return w;
  }

  static void convertGoals(const bio_ik_msgs::msg::IKRequest& ik_request,
                           bio_ik::BioIKKinematicsQueryOptions& ik_options) {
    for (auto& m : ik_request.position_goals) {
      ik_options.goals.emplace_back(new bio_ik::PositionGoal(m.link_name, p(m.position), w(m.weight)));
    }

    for (auto& m : ik_request.orientation_goals) {
      ik_options.goals.emplace_back(new bio_ik::OrientationGoal(m.link_name, q(m.orientation), w(m.weight)));
    }

    for (auto& m : ik_request.pose_goals) {
      auto* g = new bio_ik::PoseGoal(m.link_name, p(m.pose.position), q(m.pose.orientation), w(m.weight));
      g->setRotationScale(w(m.rotation_scale, 0.5));
      ik_options.goals.emplace_back(g);
    }

    for (auto& m : ik_request.look_at_goals) {
      ik_options.goals.emplace_back(new bio_ik::LookAtGoal(m.link_name, p(m.axis), p(m.target), w(m.weight)));
    }

    for (auto& m : ik_request.min_distance_goals) {
      ik_options.goals.emplace_back(new bio_ik::MinDistanceGoal(m.link_name, p(m.target), m.distance, w(m.weight)));
    }

    for (auto& m : ik_request.max_distance_goals) {
      ik_options.goals.emplace_back(new bio_ik::MaxDistanceGoal(m.link_name, p(m.target), m.distance, w(m.weight)));
    }

    for (auto& m : ik_request.line_goals) {
      ik_options.goals.emplace_back(new bio_ik::LineGoal(m.link_name, p(m.position), p(m.direction), w(m.weight)));
    }

    for (auto& m : ik_request.avoid_joint_limits_goals) {
      ik_options.goals.emplace_back(new bio_ik::AvoidJointLimitsGoal(w(m.weight), !m.primary));
    }

    for (auto& m : ik_request.minimal_displacement_goals) {
      ik_options.goals.emplace_back(new bio_ik::MinimalDisplacementGoal(w(m.weight), !m.primary));
    }

    for (auto& m : ik_request.center_joints_goals) {
      ik_options.goals.emplace_back(new bio_ik::CenterJointsGoal(w(m.weight), !m.primary));
    }

    for (auto& m : ik_request.joint_variable_goals) {
      ik_options.goals.emplace_back(
          new bio_ik::JointVariableGoal(m.variable_name, m.variable_position, w(m.weight), m.secondary));
    }

    for (auto& m : ik_request.balance_goals) {
      auto* g = new bio_ik::BalanceGoal(p(m.target), w(m.weight));
      if (m.axis.x || m.axis.y || m.axis.z) {
        g->setAxis(p(m.axis));
      }
      ik_options.goals.emplace_back(g);
    }

    for (auto& m : ik_request.side_goals) {
      ik_options.goals.emplace_back(new bio_ik::SideGoal(m.link_name, p(m.axis), p(m.direction), w(m.weight)));
    }

    for (auto& m : ik_request.direction_goals) {
      ik_options.goals.emplace_back(new bio_ik::DirectionGoal(m.link_name, p(m.axis), p(m.direction), w(m.weight)));
    }

    for (auto& m : ik_request.cone_goals) {
      ik_options.goals.emplace_back(new bio_ik::ConeGoal(m.link_name, p(m.position), w(m.position_weight), p(m.axis),
                                                         p(m.direction), m.angle, w(m.weight)));
    }
  }
};

PYBIND11_MODULE(libbitbots_moveit_bindings, m) {
  py::class_<BitbotsMoveitBindings, std::shared_ptr<BitbotsMoveitBindings>>(m, "BitbotsMoveitBindings")
      .def(py::init<std::string, std::vector<py::bytes>>())
      .def("getPositionIK", &BitbotsMoveitBindings::getPositionIK)
      .def("getPositionFK", &BitbotsMoveitBindings::getPositionFK)
      .def("getBioIKIK", &BitbotsMoveitBindings::getBioIKIK)
      .def("set_head_motors", &BitbotsMoveitBindings::setHeadMotors,
           "Set the current pan and tilt joint values [radian]", py::arg("pan"), py::arg("tilt"))
      .def("set_joint_states", &BitbotsMoveitBindings::setJointStates, "Set the current joint states")
      .def("check_collision", &BitbotsMoveitBindings::checkCollision, "Returns true if the head collides, else false");
}
