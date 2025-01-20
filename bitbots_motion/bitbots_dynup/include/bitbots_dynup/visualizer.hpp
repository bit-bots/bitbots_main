#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_

#include <bitbots_dynup/dynup_utils.hpp>
#include <bitbots_dynup/msg/dynup_ik_offset.hpp>
#include <bitbots_splines/abstract_ik.hpp>
#include <bitbots_splines/abstract_visualizer.hpp>
#include <bitbots_splines/smooth_spline.hpp>
#include <bitbots_splines/spline_container.hpp>
#include <dynup_parameters.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace bitbots_dynup {

class Visualizer : bitbots_splines::AbstractVisualizer {
 public:
  Visualizer(rclcpp::Node::SharedPtr node, bitbots_dynup::Params::Visualizer params, const std::string &base_topic);

  void setParams(bitbots_dynup::Params::Visualizer params);

  void displaySplines(bitbots_splines::PoseSpline splines, const std::string &frame);
  void publishIKOffsets(const moveit::core::RobotModelPtr &model, const DynupResponse &response,
                        const bitbots_splines::JointGoals &ik_joint_goals);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr spline_publisher_;
  rclcpp::Publisher<bitbots_dynup::msg::DynupIkOffset>::SharedPtr ik_debug_publisher_;
  std::string base_topic_;
  const std::string marker_ns_ = "bitbots_dynup";
  bitbots_dynup::Params::Visualizer params_;
};
}  // namespace bitbots_dynup

#endif  // BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_VISUALIZER_H_
