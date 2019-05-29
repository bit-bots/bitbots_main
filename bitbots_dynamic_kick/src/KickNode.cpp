#include "bitbots_dynamic_kick/KickNode.h"

KickNode::KickNode() :
        m_server(m_node_handle, "dynamic_kick", boost::bind(&KickNode::execute_cb, this, _1), false),
        m_listener(m_tf_buffer) {
    m_joint_goal_publisher = m_node_handle.advertise<bitbots_msgs::JointCommand>("kick_motor_goals", 1);
    m_server.start();
}

void KickNode::reconfigure_callback(bitbots_dynamic_kick::DynamicKickConfig &config, uint32_t level) {
    m_engine_rate = config.engine_rate;

    KickParams params = KickParams();
    params.foot_rise = config.foot_rise;
    params.foot_distance = config.foot_distance;
    params.kick_windup_distance = config.kick_windup_distance;
    params.move_trunk_time = config.move_trunk_time;
    params.raise_foot_time = config.raise_foot_time;
    params.move_to_ball_time = config.move_to_ball_time;
    params.kick_time = config.kick_time;
    params.move_back_time = config.move_back_time;
    params.lower_foot_time = config.lower_foot_time;
    params.stabilizing_point_x = config.stabilizing_point_x;
    params.stabilizing_point_y = config.stabilizing_point_y;
    m_engine.set_params(params);

    m_engine.m_stabilizer.use_minimal_displacement(config.minimal_displacement);
    m_engine.m_stabilizer.use_stabilizing(config.stabilizing);
    m_engine.m_stabilizer.set_stabilizing_weight(config.stabilizing_weight);
    m_engine.m_stabilizer.set_flying_weight(config.flying_weight);
}

void KickNode::execute_cb(const bitbots_msgs::KickGoalConstPtr &goal) {
    // TODO: maybe switch to goal callback to be able to reject goals properly
    ROS_INFO("Accepted new goal");
    m_engine.reset();

    geometry_msgs::Pose r_foot_pose;
    if (get_foot_poses(r_foot_pose, goal->ball_position.header.stamp)) {

        /* Set engines goal and start calculating */
        m_engine.set_goal(goal->ball_position,
                          goal->kick_movement,
                          r_foot_pose);
        loop_engine();

        /* Figure out the reason why loop_engine() returned and act accordingly */
        if (m_server.isPreemptRequested()) {
            /* Confirm that we canceled the previous goal */
            ROS_INFO("Cancelled old goal");
            bitbots_msgs::KickResult result;
            result.result = bitbots_msgs::KickResult::ABORTED;
            m_server.setPreempted(result);
        } else {
            /* Publish results */
            ROS_INFO("Done kicking ball");
            bitbots_msgs::KickResult result;
            result.result = bitbots_msgs::KickResult::SUCCESS;
            m_server.setSucceeded();
        }
    } else {
        /* Feet positions were not successfully retrieved */
        bitbots_msgs::KickResult result;
        result.result = bitbots_msgs::KickResult::REJECTED;
        m_server.setAborted(result, "Transformation of feet into base_link not possible");
    }
}

bool KickNode::get_foot_poses(geometry_msgs::Pose &r_foot_pose, ros::Time time) {
    /* Construct zero-positions for both feet in their respective local frames */
    geometry_msgs::PoseStamped r_foot_pose_stamped, r_foot_origin;
    r_foot_origin.header.frame_id = "r_sole";
    r_foot_origin.pose.orientation.w = 1;
    r_foot_origin.header.stamp = time;

    /* Lookup transform for both feet into base_link */
    geometry_msgs::TransformStamped trunk_transform, r_foot_transform;
    try {
        r_foot_transform = m_tf_buffer.lookupTransform("l_sole", "r_sole", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    /* Do transformation of both feet into base_link with previously retrieved transform */
    tf2::doTransform(r_foot_origin, r_foot_pose_stamped, r_foot_transform);

    /* Set result */
    r_foot_pose = r_foot_pose_stamped.pose;
    return true;
}

void KickNode::loop_engine() {
    /* Do the loop as long as nothing cancels it */
    while (m_server.isActive() && !m_server.isPreemptRequested()) {
        if (std::optional<JointGoals> goals = m_engine.tick(1.0 / m_engine_rate)) {
            // TODO: add counter for failed ticks
            bitbots_msgs::KickFeedback feedback;
            feedback.percent_done = m_engine.get_percent_done();
            feedback.chosen_foot = m_engine.is_left_kick() ?
                                   bitbots_msgs::KickFeedback::FOOT_LEFT : bitbots_msgs::KickFeedback::FOOT_RIGHT;
            m_server.publishFeedback(feedback);
            publish_goals(goals.value());
            // TODO Publish support-foot

            if (feedback.percent_done == 100) {
                break;
            }
        }

        /* Let ROS do some important work of its own and sleep afterwards */
        ros::spinOnce();
        ros::Rate loop_rate(m_engine_rate);
        loop_rate.sleep();
    }
}

void KickNode::publish_goals(const JointGoals &goals) {
    /* Construct JointCommand message */
    bitbots_msgs::JointCommand command;
    command.header.stamp = ros::Time::now();

    /*
     * Since our JointGoals type is a vector of strings
     *  combined with a vector of numbers (motor name -> target position)
     *  and bitbots_msgs::JointCommand needs both vectors as well,
     *  we can just assign them
     */
    command.joint_names = goals.first;
    command.positions = goals.second;

    /* And because we are setting position goals and not movement goals, these vectors are set to -1.0*/
    std::vector<double> vels(goals.first.size(), -1.0);
    std::vector<double> accs(goals.first.size(), -1.0);
    std::vector<double> pwms(goals.first.size(), -1.0);
    command.velocities = vels;
    command.accelerations = accs;
    command.max_currents = pwms;

    m_joint_goal_publisher.publish(command);
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
