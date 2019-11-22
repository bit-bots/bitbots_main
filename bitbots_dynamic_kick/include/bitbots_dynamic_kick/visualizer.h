//
// Created by ftsell on 6/19/19.
//

#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_VISUALIZER_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_VISUALIZER_H_

#include <bitbots_msgs/KickGoal.h>
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <bitbots_splines/smooth_spline.h>
#include <bitbots_splines/spline_container.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Vector3.h>
#include <bitbots_splines/abstract_visualizer.h>

namespace bitbots_dynamic_kick {

enum MarkerIDs {
  RECEIVED_GOAL = 1,
  KICK_WINDUP_POINT = 2,
  KICK_STABILIZING_POINT = 3,
};

struct VisualizationParams {
  int spline_smoothness;
};

class Visualizer : bitbots_splines::AbstractVisualizer {
 public:

  explicit Visualizer(const std::string &base_topic);

  void setParams(VisualizationParams params);

  void displayReceivedGoal(const bitbots_msgs::KickGoalConstPtr &goal);

  void displayFlyingSplines(bitbots_splines::PoseSpline splines, const std::string &support_foot_frame);

  void displayTrunkSplines(bitbots_splines::PoseSpline splines);

  void displayWindupPoint(const tf2::Vector3 &kick_windup_point, const std::string &support_foot_frame);

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher goal_publisher_;
  ros::Publisher foot_spline_publisher_;
  ros::Publisher trunk_spline_publisher_;
  ros::Publisher windup_publisher_;
  std::string base_topic_;
  const std::string marker_ns_ = "bitbots_dynamic_kick";
  VisualizationParams params_;
};
}

#endif //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_VISUALIZER_H_
