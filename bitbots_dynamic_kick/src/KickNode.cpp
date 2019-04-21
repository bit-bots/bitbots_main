#include "bitbots_dynamic_kick/KickNode.h"

KickNode::KickNode() :
        m_server(m_node_handle, "dynamic_kick", boost::bind(&KickNode::execute_cb, this, _1), false),
        m_listener(m_tf_buffer) {
    m_server.start();
}

void KickNode::reconfigure_callback(bitbots_dynamic_kick::DynamicKickConfig &config, uint32_t level) {
    m_engine_rate = config.engine_rate;
}

void KickNode::execute_cb(const bitbots_msgs::KickGoalConstPtr &goal) {
    // TODO: maybe switch to goal callback to be able to reject goals properly
    ROS_INFO("Accepted new goal");
    m_engine.reset();

    /* Only do actual actual kicking once required information is retrieved */
    geometry_msgs::Pose transformed_goal;
    if (transform_goal(goal->foot_target, transformed_goal)) {
        geometry_msgs::Pose l_foot_pose, r_foot_pose;
        if (get_foot_poses(l_foot_pose, r_foot_pose, goal->foot_target.header.stamp)) {

            m_engine.set_goal(transformed_goal, goal->foot_speed, l_foot_pose, r_foot_pose);
            loop_engine();

            /* Figure out the reason why loop_engine() returned and act accordingly */
            if (m_server.isPreemptRequested()) {
                /* Confirm that we canceled processing of the current goal */
                ROS_INFO("Cancelled old goal");
                m_server.setPreempted();  // TODO: return aborted result
            } else {
                /* Publish results */
                ROS_INFO("Done kicking ball");
                m_server.setSucceeded();
            }
        }
        else {
            /* get_foot_poses() was not successful */
            bitbots_msgs::KickResult result;
            result.result = bitbots_msgs::KickResult::REJECTED;
            m_server.setAborted(result, "Transformation of feet into base_link not possible");
        }
    } else {
        /* transform_goal() was not successful */
        bitbots_msgs::KickResult result;
        result.result = bitbots_msgs::KickResult::REJECTED;
        m_server.setAborted(result, "Transformation of goal into base_link not possible");
    }
}

bool KickNode::transform_goal(const geometry_msgs::PoseStamped& pose, geometry_msgs::Pose& transformed_pose) {
    /* Lookup transform from pose's frame to base_link */
    geometry_msgs::TransformStamped goal_transform;
    try {
        goal_transform = m_tf_buffer.lookupTransform("base_link", pose.header.frame_id, ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    /* Do transform pose into base_link with previously retrieved transform */
    geometry_msgs::PoseStamped transformed_pose_stamped;
    tf2::doTransform(pose, transformed_pose_stamped, goal_transform);

    /* Set result */
    transformed_pose = transformed_pose_stamped.pose;
    return true;
}

bool KickNode::get_foot_poses(geometry_msgs::Pose &l_foot_pose, geometry_msgs::Pose &r_foot_pose, ros::Time time) {
    /* Construct zero-positions for both feet in their respective local frames */
    geometry_msgs::PoseStamped l_foot_pose_stamped, r_foot_pose_stamped, l_foot_origin, r_foot_origin;
    l_foot_origin.header.frame_id = "l_foot";
    l_foot_origin.header.stamp = time;
    l_foot_origin.pose.orientation.w = 1;
    r_foot_origin.header.frame_id = "r_foot";
    r_foot_origin.pose.orientation.w = 1;
    r_foot_origin.header.stamp = time;

    /* Lookup transform for both feet into base_link */
    geometry_msgs::TransformStamped l_foot_transform, r_foot_transform;
    try {
        l_foot_transform = m_tf_buffer.lookupTransform("base_link", "l_foot", ros::Time(0), ros::Duration(1.0));
        r_foot_transform = m_tf_buffer.lookupTransform("base_link", "r_foot", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    /* Do transform both feet into base_link with previously retrieved transform */
    tf2::doTransform(l_foot_origin, l_foot_pose_stamped, l_foot_transform);
    tf2::doTransform(r_foot_origin, r_foot_pose_stamped, r_foot_transform);

    /* Set result */
    l_foot_pose = l_foot_pose_stamped.pose;
    r_foot_pose = r_foot_pose_stamped.pose;
    return true;
}

void KickNode::loop_engine() {
    /* Do the loop as long as nothing cancels it */
    while (m_server.isActive() && !m_server.isPreemptRequested()) {
        JointGoals goals;
        if (m_engine.tick(1.0 / m_engine_rate, goals)) {
            // TODO: add counter for failed ticks
            bitbots_msgs::KickFeedback feedback;
            feedback.percent_done = 0;
            feedback.chosen_foot = m_engine.is_left_kick() ?
                                   bitbots_msgs::KickFeedback::LEFT : bitbots_msgs::KickFeedback::RIGHT;
            m_server.publishFeedback(feedback);

            if (feedback.percent_done == 100) {
                break;
            }
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

    ROS_INFO("Initialized dynamic kick and waiting for actions");
    ros::spin();
}
