#ifndef BITBOTS_DYNAMIC_KICK_KICK_NODE_H
#define BITBOTS_DYNAMIC_KICK_KICK_NODE_H

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <actionlib/server/simple_action_server.h>
#include <bitbots_dynamic_kick/DynamicKickConfig.h>
#include <bitbots_msgs/KickAction.h>

typedef actionlib::SimpleActionServer<bitbots_msgs::KickAction> ActionServer;

class KickNode {
public:
    KickNode();
    void reconfigure_callback(bitbots_dynamic_kick::DynamicKickConfig &config, uint32_t level);
    void execute(const bitbots_msgs::KickGoalConstPtr& goal);
    void run();
private:
    ros::NodeHandle nh;
    ActionServer server;
};

#endif  // BITBOTS_DYNAMIC_KICK_KICK_NODE_H
