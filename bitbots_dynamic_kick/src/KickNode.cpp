#include "bitbots_dynamic_kick/KickNode.h"

KickNode::KickNode() {

}

void KickNode::reconfigure_callback(bitbots_dynamic_kick::DynamicKickConfig &config, uint32_t level) {

}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "dynamic_kick");
    KickNode node;
    dynamic_reconfigure::Server<bitbots_dynamic_kick::DynamicKickConfig> server;
    dynamic_reconfigure::Server<bitbots_dynamic_kick::DynamicKickConfig>::CallbackType f;
    f = boost::bind(&KickNode::reconfigure_callback, &node, _1, _2);
    server.setCallback(f);
}
