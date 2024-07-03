#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include <bitbots_dynup/msg/dynup_poses.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <bitbots_utils/utils.hpp>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/char.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bitbots_dynup/dynup_engine.hpp"
#include "bitbots_dynup/dynup_ik.hpp"
#include "bitbots_dynup/dynup_stabilizer.hpp"
#include "bitbots_dynup/visualizer.hpp"
#include "bitbots_msgs/action/dynup.hpp"
#include "dynup_parameters.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace bitbots_dynup {
using DynupGoal = bitbots_msgs::action::Dynup;
using DynupGoalHandle = rclcpp_action::ServerGoalHandle<DynupGoal>;
using namespace std::placeholders;

/**
 * DynupNode is that part of bitbots_dynamic_DynUp which takes care of interacting with ROS and utilizes a DynUpEngine
 * to calculate actual DynUp behavior.
 *
 * It provides an ActionServer for the bitbots_msgs::DynUpAction.
 * This actionServer accepts new goals in any tf frame, and sets up the DynUpEngines to work towards this new goal
 *
 * Additionally it publishes the DynUpEngines motor-goals back into ROS
 */
class DynupNode {
 public:
  explicit DynupNode(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                     std::vector<rclcpp::Parameter> parameters = {});

  void onSetParameters();

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_states);

  DynupEngine *getEngine();

  DynupIK *getIK();

  /**
   * Retrieve current positions of left foot and trunk relative to right foot
   *
   * @return The pair of (right foot, left foot) poses if transformation was successful
   */
  bitbots_dynup::msg::DynupPoses getCurrentPoses();

  bitbots_msgs::msg::JointCommand step(double dt);

  bitbots_msgs::msg::JointCommand step(double dt, const sensor_msgs::msg::Imu::SharedPtr imu_msg, const sensor_msgs::msg::JointState::SharedPtr joint_state_msg);

  geometry_msgs::msg::PoseArray step_open_loop(double dt);

  void reset(int time = 0);

 private:
  /**
   * Callback that gets executed whenever #m_server receives a new goal.
   * @param goal New goal to process
   */
  rclcpp_action::GoalResponse goalCb(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const DynupGoal::Goal> goal);

  rclcpp_action::CancelResponse cancelCb(std::shared_ptr<DynupGoalHandle> goal);

  void acceptedCb(const std::shared_ptr<DynupGoalHandle> goal);

  // Store reference to the "real" node
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Server<DynupGoal>::SharedPtr action_server_;

  std::vector<std::string> param_names_;

  // Declare parameter listener and struct from the generate_parameter_library
  bitbots_dynup::ParamListener param_listener_;
  // Data structure to hold all parameters, which is build from the schema in the 'parameters.yaml'
  bitbots_dynup::Params params_;

  // Subcomponents of the dynup
  DynupEngine engine_;
  Stabilizer stabilizer_;
  Visualizer visualizer_;
  DynupIK ik_;

  // Variables for node
  int stable_duration_ = 0;
  int failed_tick_counter_ = 0;
  double last_ros_update_time_ = 0;
  double start_time_ = 0;
  bool server_free_ = true;
  bool debug_ = false;

  // TF2 related things
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // MoveIt related things
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;

  // Publishers and subscribers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_publisher_;
  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr joint_goal_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  void execute(const std::shared_ptr<DynupGoalHandle> goal);

  /**
   * Do main loop in which DynUpEngine::tick() gets called repeatedly.
   * The ActionServer's state is taken into account meaning that a cancelled goal no longer gets processed.
   */
  void loopEngine(int, std::shared_ptr<DynupGoalHandle> goal_handle);

  /**
   * Publish the current support_foot so that a correct base_footprint can be calculated
   * @param is_left_dyn_up Whether the left foot is the current DynUping foot, meaning it is in the air
   */
  void publishSupportFoot(bool is_left_dyn_up);

  /**
   * Creates the Goal Msg
   */
  bitbots_msgs::msg::JointCommand createGoalMsg(const bitbots_splines::JointGoals &goals);

  /**
   * Helper method to achieve correctly sampled rate
   */
  double getTimeDelta();
};

}  // namespace bitbots_dynup

#endif  // BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_
