#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_NODE_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_NODE_H_

#include <string>
#include <optional>
#include <ros/ros.h>
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <bitbots_msgs/KickAction.h>
#include <bitbots_msgs/JointCommand.h>
#include <bitbots_dynamic_kick/DynamicKickConfig.h>
#include <std_msgs/Char.h>
#include "bitbots_dynamic_kick/kick_engine.h"
#include "bitbots_dynamic_kick/visualizer.h"
#include "bitbots_dynamic_kick/kick_ik.h"

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
  KickNode();

  /** Callback for dynamic reconfigure */
  void reconfigureCallback(bitbots_dynamic_kick::DynamicKickConfig &config, uint32_t level);

  /**
   * Callback that gets executed whenever #m_server receives a new goal.
   * @param goal New goal to process
   */
  void executeCb(const bitbots_msgs::KickGoalConstPtr &goal);

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher joint_goal_publisher_;
  ros::Publisher support_foot_publisher_;
  ros::Subscriber cop_l_subscriber_;
  ros::Subscriber cop_r_subscriber_;
  ActionServer server_;
  KickEngine engine_;
  Stabilizer stabilizer_;
  Visualizer visualizer_;
  KickIK ik_;
  int engine_rate_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;

  /**
   * Do main loop in which KickEngine::update() gets called repeatedly.
   * The ActionServer's state is taken into account meaning that a cancelled goal no longer gets processed.
   */
  void loopEngine();

  /**
   * Retrieve current feet_positions in base_link frame
   *
   * @return The pair of (right foot, left foot) poses if transformation was successfull
   */
  std::optional<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> getFootPoses();

  /**
   * Publish the current support_foot so that a correct base_footprint can be calculated
   * @param is_left_kick Whether the left foot is the current kicking foot, meaning it is in the air
   */
  void publishSupportFoot(bool is_left_kick);

  /**
   * Publish goals to ROS
   */
  void publishGoals(const bitbots_splines::JointGoals &goals);
  void copLCallback(const geometry_msgs::PointStamped& cop);
  void copRCallback(const geometry_msgs::PointStamped& cop);
};
}

#endif  //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_KICK_NODE_H_
