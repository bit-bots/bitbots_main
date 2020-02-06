#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_

#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <bitbots_splines/smooth_spline.h>
#include <bitbots_splines/spline_container.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Vector3.h>
#include <bitbots_splines/abstract_visualizer.h>

namespace bitbots_dynup {

struct VisualizationParams {
  int spline_smoothness;
};

class Visualizer : bitbots_splines::AbstractVisualizer {
 public:

  Visualizer(const std::string &base_topic);

  void setParams(VisualizationParams params);

  void displaySplines(bitbots_splines::PoseSpline splines, const std::string &frame, const int id);

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher spline_publisher_;
  std::string base_topic_;
  const std::string marker_ns_ = "bitbots_dynup";
  VisualizationParams params_;
};
}

#endif //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_
