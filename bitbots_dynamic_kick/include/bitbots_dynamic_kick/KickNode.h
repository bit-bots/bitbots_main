#ifndef BITBOTS_DYNAMIC_KICK_KICK_NODE_H
#define BITBOTS_DYNAMIC_KICK_KICK_NODE_H

#include <string>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <bitbots_msgs/KickAction.h>
#include <bitbots_dynamic_kick/DynamicKickConfig.h>
#include "bitbots_dynamic_kick/KickEngine.h"

typedef actionlib::SimpleActionServer<bitbots_msgs::KickAction> ActionServer;

/**
 * KickNode is that part of bitbots_dynamic_kick which takes care of interacting with ROS and utilizes KickEngine
 * to calculate actual kick behavior.
 *
 * It provides an ActionServer for the bitbots_msgs::KickAction
 */
class KickNode {
public:
    KickNode();

    /** Callback for dynamic reconfigure */
    void reconfigure_callback(bitbots_dynamic_kick::DynamicKickConfig &config, uint32_t level);

    /**
     * Callback that gets executed whenever m_server receives a new goal.
     * @param goal New goal to process
     */
    void execute_cb(const bitbots_msgs::KickGoalConstPtr& goal);
private:
    ros::NodeHandle m_node_handle;
    ActionServer m_server;
    KickEngine m_engine;
    int m_engine_rate;

    /**
     * Do main loop in which engine.tick() get called repeatedly.
     * The ActionServers state is taken into account meaning that a cancelled goal no longer get processed.
     */
    void loop_engine();
};

#endif  // BITBOTS_DYNAMIC_KICK_KICK_NODE_H
