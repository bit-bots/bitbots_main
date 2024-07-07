#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_

#include <tf2/LinearMath/Vector3.h>

#include <bitbots_splines/abstract_visualizer.hpp>
#include <bitbots_splines/smooth_spline.hpp>
#include <bitbots_splines/spline_container.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <visualization_msgs/msg/marker_array.hpp>

#include "dynup_parameters.hpp"

namespace bitbots_dynup {

class Visualizer : bitbots_splines::AbstractVisualizer {
 public:
  Visualizer(rclcpp::Node::SharedPtr node, bitbots_dynup::Params::Visualizer params, const std::string &base_topic);

  void setParams(bitbots_dynup::Params::Visualizer params);

  void displaySplines(bitbots_splines::PoseSpline splines, const std::string &frame);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr spline_publisher_;
  std::string base_topic_;
  const std::string marker_ns_ = "bitbots_dynup";
  bitbots_dynup::Params::Visualizer params_;
};
}  // namespace bitbots_dynup

#endif  // BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_
