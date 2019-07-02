#include "bitbots_dynamic_kick/Visualizer.h"


Visualizer::Visualizer(std::string base_topic) :
        m_base_topic(base_topic),
        m_params() {
    /* make sure m_base_topic has consistent scheme */
    if (base_topic.compare(base_topic.size() - 1, 1, "/") != 0) {
        m_base_topic += "/";
    }

    /* create necessary publishers */
    m_goal_publisher = m_node_handle.advertise<visualization_msgs::Marker>(m_base_topic + "received_goal",
            /* queue_size */ 5, /* latch */ true);
    m_spline_publisher = m_node_handle.advertise<nav_msgs::Path>(m_base_topic + "flying_foot_spline",
            /* queue_size */ 5, /* latch */ true);
    m_windup_publisher = m_node_handle.advertise<visualization_msgs::Marker>(m_base_topic + "kick_windup_point",
            /* queue_size */ 5, /* latch */ true);
    m_stabilizing_publisher = m_node_handle.advertise<visualization_msgs::Marker>(m_base_topic + "kick_stabilizing_point",
            /* queue_size */ 5, /* latch */ true);

    m_node_handle.getParam("/debug_active", m_param_debug_active);
}


void Visualizer::set_params(VisualizationParams params) {
    m_params = params;
}


void Visualizer::display_flying_splines(Trajectories &splines, std::string support_foot_frame) {
    if (!is_enabled())
        return;

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = support_foot_frame;

    for (int i = 0; i < splines.size() * m_params.spline_smoothnes; i++) {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = path.header.stamp;
        pose.header.frame_id = support_foot_frame;
        pose.pose.position.x = splines.get("pos_x").pos((float) i / (float) m_params.spline_smoothnes);
        pose.pose.position.y = splines.get("pos_y").pos((float) i / (float) m_params.spline_smoothnes);
        pose.pose.position.z = splines.get("pos_z").pos((float) i / (float) m_params.spline_smoothnes);
        pose.pose.orientation.w = 1;

        path.poses.push_back(pose);
    }

    m_spline_publisher.publish(path);
}


void Visualizer::display_received_goal(const bitbots_msgs::KickGoalConstPtr &goal) {
    if (!is_enabled())
        return;

    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = MarkerIDs::received_goal;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.lifetime = ros::Duration(1000);
    marker.frame_locked = false;
    marker.header.frame_id = goal->header.frame_id;
    marker.header.stamp = goal->header.stamp;
    marker.pose.position.x = goal->ball_position.x;
    marker.pose.position.y = goal->ball_position.y;
    marker.pose.position.z = goal->ball_position.z;
    marker.pose.orientation = goal->kick_direction;
    marker.scale.x = 0.08 + (goal->kick_speed / 3);
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1;
    marker.color.r = 1;

    m_goal_publisher.publish(marker);
}


void Visualizer::display_windup_point(tf2::Vector3 kick_windup_point, std::string support_foot_frame) {
    if (!is_enabled())
        return;

    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = MarkerIDs::received_goal;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(1000);
    marker.frame_locked = false;
    marker.header.frame_id = support_foot_frame;
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = kick_windup_point.x();
    marker.pose.position.y = kick_windup_point.y();
    marker.pose.position.z = kick_windup_point.z();
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1;
    marker.color.g = 1;

    m_windup_publisher.publish(marker);
}


void Visualizer::display_stabilizing_point(tf::Vector3 kick_windup_point, std::string support_foot_frame) {
    if (!is_enabled())
        return;

    visualization_msgs::Marker marker;

    marker.ns = m_marker_ns;
    marker.id = MarkerIDs::kick_stabilizing_point;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(1000);
    marker.frame_locked = false;
    marker.header.frame_id = support_foot_frame;
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = kick_windup_point.x();
    marker.pose.position.y = kick_windup_point.y();
    marker.pose.position.z = kick_windup_point.z();
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1;
    marker.color.g = 1;

    m_stabilizing_publisher.publish(marker);
}


bool Visualizer::is_enabled() {
    return m_params.force_enable || m_param_debug_active;
}
