#include "bitbots_dynamic_kick/KickNode.h"

KickNode::KickNode() :
        m_server(m_node_handle, "dynamic_kick", boost::bind(&KickNode::execute_cb, this, _1), false) {
    m_server.start();
}

void KickNode::reconfigure_callback(bitbots_dynamic_kick::DynamicKickConfig &config, uint32_t level) {
    m_engine_rate = config.engine_rate;
}

void KickNode::execute_cb(const bitbots_msgs::KickGoalConstPtr &goal) {
    /* Setup engine for new goal and start calculating */
    m_engine.reset();
    m_engine.set_goal(goal);
    loop_engine();

    /* Figure out the reason why loop_engine() returned and act accordingly */
    if (m_server.isPreemptRequested()) {
        /* Confirm that we canceled processing of the current goal */
        m_server.setPreempted();
    } else {
        /* Publish results */
        m_server.setSucceeded();
    }
}

void KickNode::loop_engine() {
    /* Do the loop as long as nothing cancels it */
    while (m_server.isActive() && !m_server.isPreemptRequested()) {
        bitbots_msgs::KickFeedback feedback = m_engine.tick();
        m_server.publishFeedback(feedback);

        if (feedback.percent_done == 100) {
            break;
        }

        /* Let ROS do some important work of its own sleep afterwards */
        ros::spinOnce();
        ros::Rate loop_rate(m_engine_rate);
        loop_rate.sleep();
    }
}

int main(int argc, char *argv[]) {
    /* Setup ROS node */
    ros::init(argc, argv, "dynamic_kick");
    KickNode node;

    /* Setup dynamic_reconfigure */
    dynamic_reconfigure::Server<bitbots_dynamic_kick::DynamicKickConfig> dyn_reconf_server;
    dynamic_reconfigure::Server<bitbots_dynamic_kick::DynamicKickConfig>::CallbackType f;
    f = boost::bind(&KickNode::reconfigure_callback, &node, _1, _2);
    dyn_reconf_server.setCallback(f);

    ros::spin();
}
