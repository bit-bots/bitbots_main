#include "bitbots_dynamic_kick/KickNode.h"
#include <iostream>

KickNode::KickNode() : server(nh, "dynamic_kick", boost::bind(&KickNode::execute, this, _1), false) {
    server.start();
}

void KickNode::run() {
}

void KickNode::reconfigure_callback(bitbots_dynamic_kick::DynamicKickConfig &config, uint32_t level) {
}

void KickNode::execute(const bitbots_msgs::KickGoalConstPtr& goal) {
    std::cout << "Received action! Speed: " << goal->foot_speed << std::endl;
    server.setSucceeded();
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "dynamic_kick");
    KickNode node;
    dynamic_reconfigure::Server<bitbots_dynamic_kick::DynamicKickConfig> server;
    dynamic_reconfigure::Server<bitbots_dynamic_kick::DynamicKickConfig>::CallbackType f;
    f = boost::bind(&KickNode::reconfigure_callback, &node, _1, _2);
    server.setCallback(f);
    node.run();
    ros::spin();
}
