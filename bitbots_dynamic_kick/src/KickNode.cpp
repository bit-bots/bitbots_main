#include "bitbots_dynamic_kick/KickNode.h"

KickNode::KickNode() :
        m_server(m_node_handle, "dynamic_kick", boost::bind(&KickNode::execute, this, _1), false) {
    m_server.start();
}

void KickNode::run() {
    while (ros::ok()) {
        ros::Rate loop_rate(m_engine_rate);

        if (m_server.isActive()) {
            bitbots_msgs::KickFeedback feedback = m_engine.tick();
            m_server.publishFeedback(feedback);

            if (feedback.percent_done == 100) {
                m_server.setSucceeded();
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void KickNode::reconfigure_callback(bitbots_dynamic_kick::DynamicKickConfig& config, uint32_t level) {
    m_engine_rate = config.engine_rate;
}

void KickNode::execute(const bitbots_msgs::KickGoalConstPtr& goal) {
    m_engine.reset();
    m_engine.set_goal(goal);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "dynamic_kick");
    KickNode node;
    dynamic_reconfigure::Server<bitbots_dynamic_kick::DynamicKickConfig> dyn_reconf_server;
    dynamic_reconfigure::Server<bitbots_dynamic_kick::DynamicKickConfig>::CallbackType f;
    f = boost::bind(&KickNode::reconfigure_callback, &node, _1, _2);
    dyn_reconf_server.setCallback(f);
    node.run();
}
