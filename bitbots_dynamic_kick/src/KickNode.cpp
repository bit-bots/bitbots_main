#include "bitbots_dynamic_kick/KickNode.h"

KickNode::KickNode() :
        m_server(m_node_handle, "dynamic_kick", boost::bind(&KickNode::execute, this, _1), false) {
    m_server.start();
}

void KickNode::reconfigure_callback(bitbots_dynamic_kick::DynamicKickConfig& config, uint32_t level) {
    m_engine_rate = config.engine_rate;
}

void KickNode::execute(const bitbots_msgs::KickGoalConstPtr& goal) {
    std::cout << "Received new goal" << std::endl;

    m_engine.reset();
    m_engine.set_goal(goal);

    loop_engine();

    if (m_server.isPreemptRequested()) {
        m_server.setPreempted();
    } else {
        m_server.setSucceeded();
    }
}

void KickNode::loop_engine() {
    while (m_server.isActive() && !m_server.isPreemptRequested()) {
        ros::Rate loop_rate(m_engine_rate);

        bitbots_msgs::KickFeedback feedback = m_engine.tick();
        m_server.publishFeedback(feedback);

        if (feedback.percent_done == 100) {
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "dynamic_kick");
    KickNode node;
    dynamic_reconfigure::Server<bitbots_dynamic_kick::DynamicKickConfig> dyn_reconf_server;
    dynamic_reconfigure::Server<bitbots_dynamic_kick::DynamicKickConfig>::CallbackType f;
    f = boost::bind(&KickNode::reconfigure_callback, &node, _1, _2);
    dyn_reconf_server.setCallback(f);
    ros::spin();
}
