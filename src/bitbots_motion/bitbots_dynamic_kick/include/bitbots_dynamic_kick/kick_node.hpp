#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_NODE_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_NODE_H_

#include <unistd.h>

#include <Eigen/Geometry>
#include <biped_interfaces/msg/phase.hpp>
#include <bitbots_dynamic_kick/kick_engine.hpp>
#include <bitbots_dynamic_kick/kick_ik.hpp>
#include <bitbots_dynamic_kick/kick_utils.hpp>
#include <bitbots_dynamic_kick/visualizer.hpp>
#include <bitbots_msgs/action/kick.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/char.hpp>
#include <string>
#include <tf2/convert.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace bitbots_dynamic_kick {
using KickGoal = bitbots_msgs::action::Kick;
using KickGoalHandle = rclcpp_action::ServerGoalHandle<KickGoal>;
using namespace std::placeholders;

/**
 * KickNode is that part of bitbots_dynamic_kick which takes care of interacting with ROS and utilizes a KickEngine
 * to calculate actual kick behavior.
 *
 * It provides an ActionServer for the bitbots_msgs::KickAction.
 * This actionServer accepts new goals in any tf frame, and sets up the KickEngines to work towards this new goal
 *
 * Additionally it publishes the KickEngines motor-goals back into ROS
 */
class KickNode : public rclcpp::Node {
 public:
  explicit KickNode(const std::string& ns = std::string(), std::vector<rclcpp::Parameter> parameters = {});

  /**
   * Callback that gets executed whenever #server_ receives a new goal.
   * @param goal New goal to process
   */
  void executeCb(const std::shared_ptr<KickGoalHandle> goal_handle);

  /**
   * This wrapper is used in the python wrapper for a single step of the kick
   * @param dt the time difference since the last call of this method
   * @return the JointCommand representing the next step or an empty JointCommand if the kick is done
   */
  bitbots_msgs::msg::JointCommand stepWrapper(double dt);

  /**
   * Get the current progress of the kick, from 0 to 1
   */
  double getProgress();

  /**
   * Initialize the node
   * @param goal_msg The goal_msg of the kick
   * @param error_string when the return value is false, this will contain details about the error
   * @param trunk_to_base_footprint transform from trunk to base_footprint
   * @return whether the setup was successful
   */
  bool init(const bitbots_msgs::action::Kick::Goal& goal_msg, std::string& error_string);

  /**
   * Set the current joint state of the robot
   */
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_states);

  /**
   * Get the current pose of the trunk, relative to the support foot
   */
  geometry_msgs::msg::Pose getTrunkPose();

  /**
   * Whether the left foot is the current kicking foot
   */
  bool isLeftKick();

 private:
  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr joint_goal_publisher_;
  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr support_foot_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cop_l_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cop_r_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp_action::Server<bitbots_msgs::action::Kick>::SharedPtr server_;
  KickEngine engine_;
  Stabilizer stabilizer_;
  Visualizer visualizer_;
  KickIK ik_;
  int engine_rate_;
  double last_ros_update_time_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  bool was_support_foot_published_;
  moveit::core::RobotStatePtr goal_state_;
  moveit::core::RobotStatePtr current_state_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  bool currently_kicking_ = false;

  std::string base_link_frame_, base_footprint_frame_, l_sole_frame_, r_sole_frame_;

  KickParams unstable_config_;
  KickParams normal_config_;

  /**
   * Do main loop in which KickEngine::update() gets called repeatedly.
   * The ActionServer's state is taken into account meaning that a cancelled goal no longer gets processed.
   */
  void loopEngine(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bitbots_msgs::action::Kick>> goal_handle);

  rclcpp_action::GoalResponse goalCb(const rclcpp_action::GoalUUID& uuid,
                                     std::shared_ptr<const bitbots_msgs::action::Kick::Goal> goal);

  rclcpp_action::CancelResponse cancelCb(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<bitbots_msgs::action::Kick>> goal);

  void acceptedCb(const std::shared_ptr<KickGoalHandle> goal);

  /**
   * Execute one step of engine-stabilize-ik
   * @return the motor goals
   */
  bitbots_splines::JointGoals kickStep(double dt);

  /**
   * Publish the current support_foot so that a correct base_footprint can be calculated
   * @param is_left_kick Whether the left foot is the current kicking foot, meaning it is in the air
   */
  void publishSupportFoot(bool is_left_kick);

  /**
   * Get JointCommand message for JointGoals
   */
  bitbots_msgs::msg::JointCommand getJointCommand(const bitbots_splines::JointGoals& goals);

  void copLCallback(const geometry_msgs::msg::PointStamped::SharedPtr cop);

  void copRCallback(const geometry_msgs::msg::PointStamped::SharedPtr cop);

  rcl_interfaces::msg::SetParametersResult onSetParameters(const std::vector<rclcpp::Parameter>& parameters);
};
}  // namespace bitbots_dynamic_kick

#endif  // BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_NODE_H_
