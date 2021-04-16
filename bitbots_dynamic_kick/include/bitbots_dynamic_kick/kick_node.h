#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_NODE_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_NODE_H_

#include <string>
#include <optional>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Char.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <bitbots_msgs/KickAction.h>
#include <bitbots_msgs/JointCommand.h>
#include <bitbots_dynamic_kick/DynamicKickConfig.h>
#include <bitbots_dynamic_kick/kick_engine.h>
#include <bitbots_dynamic_kick/visualizer.h>
#include <bitbots_dynamic_kick/kick_ik.h>
#include <bitbots_dynamic_kick/kick_utils.h>
#include <bitbots_msgs/SupportState.h>

namespace bitbots_dynamic_kick {

typedef actionlib::SimpleActionServer<bitbots_msgs::KickAction> ActionServer;

/**
 * KickNode is that part of bitbots_dynamic_kick which takes care of interacting with ROS and utilizes a KickEngine
 * to calculate actual kick behavior.
 *
 * It provides an ActionServer for the bitbots_msgs::KickAction.
 * This actionServer accepts new goals in any tf frame, and sets up the KickEngines to work towards this new goal
 *
 * Additionally it publishes the KickEngines motor-goals back into ROS
 */
class KickNode {
 public:
  explicit KickNode(const std::string &ns = std::string());

  /** Callback for dynamic reconfigure */
  void reconfigureCallback(bitbots_dynamic_kick::DynamicKickConfig &config, uint32_t level);

  /**
   * Callback that gets executed whenever #server_ receives a new goal.
   * @param goal New goal to process
   */
  void executeCb(const bitbots_msgs::KickGoalConstPtr &goal);

  /**
   * This wrapper is used in the python wrapper for a single step of the kick
   * @param dt the time difference since the last call of this method
   * @return the JointCommand representing the next step or an empty JointCommand if the kick is done
   */
  bitbots_msgs::JointCommand stepWrapper(double dt);

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
  bool init(const bitbots_msgs::KickGoal &goal_msg, std::string &error_string);

  /**
   * Set the current joint state of the robot
   */
  void jointStateCallback(const sensor_msgs::JointState &joint_states);

  /**
   * Get the current pose of the trunk, relative to the support foot
   */
  geometry_msgs::Pose getTrunkPose();
 private:
  ros::NodeHandle node_handle_;
  ros::Publisher joint_goal_publisher_;
  ros::Publisher support_foot_publisher_;
  ros::Subscriber cop_l_subscriber_;
  ros::Subscriber cop_r_subscriber_;
  ros::Subscriber joint_state_subscriber_;
  ActionServer server_;
  KickEngine engine_;
  Stabilizer stabilizer_;
  Visualizer visualizer_;
  KickIK ik_;
  int engine_rate_;
  double last_ros_update_time_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  robot_model_loader::RobotModelLoader robot_model_loader_;
  bool was_support_foot_published_;
  robot_state::RobotStatePtr goal_state_;
  robot_state::RobotStatePtr current_state_;

  std::string base_link_frame_, base_footprint_frame_, l_sole_frame_, r_sole_frame_;

    /**
   * Do main loop in which KickEngine::update() gets called repeatedly.
   * The ActionServer's state is taken into account meaning that a cancelled goal no longer gets processed.
   */
  void loopEngine(ros::Rate loop_rate);

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
   * Helper method to achieve correctly sampled rate
   */
  double getTimeDelta();

  /**
   * Get JointCommand message for JointGoals
   */
  bitbots_msgs::JointCommand getJointCommand(const bitbots_splines::JointGoals &goals);
  void copLCallback(const geometry_msgs::PointStamped &cop);
  void copRCallback(const geometry_msgs::PointStamped &cop);
};
}

#endif  //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_NODE_H_
