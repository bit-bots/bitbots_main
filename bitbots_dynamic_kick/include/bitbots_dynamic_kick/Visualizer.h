//
// Created by ftsell on 6/19/19.
//

#ifndef BITBOTS_DYNAMIC_KICK_VISUALIZER_H
#define BITBOTS_DYNAMIC_KICK_VISUALIZER_H

#include <bitbots_msgs/KickGoal.h>
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <bitbots_splines/SmoothSpline.hpp>
#include <bitbots_splines/SplineContainer.hpp>
#include <geometry_msgs/PoseStamped.h>

typedef bitbots_splines::SplineContainer<bitbots_splines::SmoothSpline> Trajectories;


namespace MarkerIDs {
    const int received_goal = 1;
}

class VisualizationParams {
public:
    bool force_enable;
};


class Visualizer {
public:

    explicit Visualizer(std::string base_topic, ros::NodeHandle &node_handle);

    void set_params(VisualizationParams params);

    void display_received_goal(const bitbots_msgs::KickGoalConstPtr &goal);

    void display_fyling_splines(Trajectories &splines, std::string support_foot_frame);

private:
    ros::NodeHandle m_node_handle;
    ros::Publisher m_goal_publisher;
    ros::Publisher m_spline_publisher;
    std::string m_base_topic;
    const std::string m_marker_ns = "bitbots_dynamic_kick";
    VisualizationParams m_params;
    bool m_param_debug_active;

    bool is_enabled();
};

#endif //BITBOTS_DYNAMIC_KICK_VISUALIZER_H
