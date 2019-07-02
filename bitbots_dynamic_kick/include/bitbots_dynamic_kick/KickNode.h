#ifndef BITBOTS_DYNAMIC_KICK_KICK_NODE_H
#define BITBOTS_DYNAMIC_KICK_KICK_NODE_H

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
#include "bitbots_dynamic_kick/KickEngine.h"
#include <std_msgs/Char.h>
#include "bitbots_dynamic_kick/Visualizer.h"

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
    void reconfigure_callback(bitbots_dynamic_kick::DynamicKickConfig &config, uint32_t level);

    /**
     * Callback that gets executed whenever #m_server receives a new goal.
     * @param goal New goal to process
     */
    void execute_cb(const bitbots_msgs::KickGoalConstPtr &goal);

private:
    ros::NodeHandle m_node_handle;
    ros::Publisher m_joint_goal_publisher;
    ros::Publisher m_support_foot_publisher;
    ros::Subscriber m_cop_l_subscriber;
    ros::Subscriber m_cop_r_subscriber;
    ActionServer m_server;
    KickEngine m_engine;
    int m_engine_rate;
    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_listener;

    /**
     * Do main loop in which KickEngine::tick() gets called repeatedly.
     * The ActionServer's state is taken into account meaning that a cancelled goal no longer gets processed.
     */
    void loop_engine();

    /**
     * Retrieve current feet_positions in base_link frame
     *
     * @return The pair of (right foot, left foot) poses if transformation was successfull
     */
    std::optional<std::pair<geometry_msgs::Pose, geometry_msgs::Pose>> get_foot_poses();

    /**
     * Publish the current support_foot so that a correct base_footprint can be calculated
     * @param is_left_kick Whether the left foot is the current kicking foot, meaning it is in the air
     */
    void publish_support_foot(bool is_left_kick);

    /**
     * Publish goals to ROS
     */
    void publish_goals(const JointGoals &goals);
    inline void cop_l_callback(const geometry_msgs::PointStamped cop);
    inline void cop_r_callback(const geometry_msgs::PointStamped cop);
};

#endif  // BITBOTS_DYNAMIC_KICK_KICK_NODE_H
