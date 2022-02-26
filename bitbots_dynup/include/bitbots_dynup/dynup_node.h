#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_

#include <cmath>
#include <string>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include <unistd.h>

#include "rclcpp_action/rclcpp_action.hpp"
#include <std_msgs/msg/char.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <bitbots_msgs/msg/joint_command.hpp>
#include <bitbots_dynup/msg/dynup_poses.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "bitbots_dynup/visualizer.h"
#include "bitbots_dynup/dynup_engine.h"
#include "bitbots_dynup/dynup_ik.h"
#include "bitbots_dynup/dynup_stabilizer.h"

namespace bitbots_dynup {

/**
 * DynupNode is that part of bitbots_dynamic_DynUp which takes care of interacting with ROS and utilizes a DynUpEngine
 * to calculate actual DynUp behavior.
 *
 * It provides an ActionServer for the bitbots_msgs::DynUpAction.
 * This actionServer accepts new goals in any tf frame, and sets up the DynUpEngines to work towards this new goal
 *
 * Additionally it publishes the DynUpEngines motor-goals back into ROS
 */
class DynupNode : public rclcpp::Node {
 public:
  explicit DynupNode(const std::string &ns = std::string(), const rclcpp::NodeOptions &options = rclcpp::NodeOptions) : Node("dynup_node", options);

  /**
   * Callback that gets executed whenever #m_server receives a new goal.
   * @param goal New goal to process
   */
  rclcpp_action::GoalResponse goalCb(const rclcpp_action::GoalUUID & uuid, const bitbots_msgs::DynUpGoalSharedPtr &goal);
  rclcpp_action::CancelResponse cancelCb(const bitbots_msgs::DynUpGoalSharedPtr &goal);
  rclcpp_action::AcceptedResponse acceptedCb(const bitbots_msgs::DynUpGoalSharedPtr &goal);


  this->action_server_ = rclcpp_action::create_server<DynupNode>(
      this,
      "dynup_node",
      std::bind(&DynupNode::goalCb, this, _1, _2),
      std::bind(&DynupNode::cancelCb, this, _1),
      std::bind(&DynupNode::acceptedCb, this, _1));

  rcl_interfaces::msg::SetParametersResult WalkNode::onSetParameters(const std::vector<rclcpp::Parameter> &parameters);

  void imuCallback(const sensor_msgs::msg::Imu &msg);

  void jointStateCallback(const sensor_msgs::msg::JointState &jointstates);

  DynupEngine *getEngine();
  DynupIK *getIK();

  /**
  * Retrieve current positions of left foot and trunk relative to right foot
  *
  * @return The pair of (right foot, left foot) poses if transformation was successfull
  */
  bitbots_dynup::msg::DynupPoses getCurrentPoses();

  bitbots_msgs::msg::JointCommand step(double dt);
  bitbots_msgs::msg::JointCommand step(double dt,
                                  const sensor_msgs::msg::Imu &imu_msg,
                                  const sensor_msgs::msg::JointState &jointstate_msg);
  geometry_msgs::msg::PoseArray step_open_loop(double dt);

  void reset(int time=0);

 private:
  rclcpp::Publisher debug_publisher_;
  
  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr joint_goal_publisher_;
  rclcpp::Subscription<DynupNode::imuCallback> cop_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  std::vector<std::string> param_names_

  ActionServer server_;
  DynupEngine engine_;
  Stabilizer stabilizer_;
  Visualizer visualizer_;
  DynupIK ik_;
  std::vector<rclcpp::Parameter> params_;
  int stable_duration_;
  int engine_rate_;
  int failed_tick_counter_;
  double last_ros_update_time_;
  double start_time_;
  bool debug_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::msg::TransformListener listener_;
  robot_model_loader::RobotModelLoader robot_model_loader_;

  std::string base_link_frame_, l_sole_frame_, r_sole_frame_, l_wrist_frame_, r_wrist_frame_;

  /**
   * Do main loop in which DynUpEngine::tick() gets called repeatedly.
   * The ActionServer's state is taken into account meaning that a cancelled goal no longer gets processed.
   */
  void loopEngine(rclcpp::Rate loop_rate);

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

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_NODE_H_
