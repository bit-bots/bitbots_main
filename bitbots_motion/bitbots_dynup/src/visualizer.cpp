#include "bitbots_dynup/visualizer.hpp"

#include <bitbots_splines/pose_spline.hpp>

namespace bitbots_dynup {
Visualizer::Visualizer(rclcpp::Node::SharedPtr node, bitbots_dynup::Params::Visualizer params,
                       const std::string &base_topic)
    : node_(node), base_topic_(base_topic), params_(params) {
  /* make sure base_topic_ has consistent scheme */
  if (base_topic.compare(base_topic.size() - 1, 1, "/") != 0) {
    base_topic_ += "/";
  }

  /* create necessary publishers */
  spline_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(base_topic_ + "received_goal",
                                                                                    /* queue_size */ 5);
}

void Visualizer::setParams(bitbots_dynup::Params::Visualizer params) { params_ = params; }

void Visualizer::displaySplines(bitbots_splines::PoseSpline splines, const std::string &frame) {
  // if (spline_publisher_->get_subscription_count() == 0)
  //     return;
  visualization_msgs::msg::MarkerArray path = getPath(splines, frame, params_.spline_smoothness, node_);

  spline_publisher_->publish(path);
}
}  // namespace bitbots_dynup
