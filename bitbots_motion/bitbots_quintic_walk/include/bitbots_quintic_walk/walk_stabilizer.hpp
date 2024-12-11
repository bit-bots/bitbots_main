#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_

#include <rot_conv/rot_conv.h>

#include <Eigen/Geometry>
#include <bitbots_splines/abstract_stabilizer.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bitbots_quintic_walk/walk_utils.hpp"
#include "bitbots_splines/abstract_stabilizer.hpp"

namespace bitbots_quintic_walk {

class WalkStabilizer : public bitbots_splines::AbstractStabilizer<WalkResponse> {
 public:
  explicit WalkStabilizer(rclcpp::Node::SharedPtr node);
  void reset() override;
  WalkResponse stabilize(const WalkResponse &response, const rclcpp::Duration &dt) override;
  WalkRequest adjust_step_length(WalkRequest request, const double imu_roll, const double imu_pitch,
                                 double pitch_threshold, double roll_threshold, const rclcpp::Duration &dt);

 private:
  rclcpp::Node::SharedPtr node_;
  control_toolbox::PidROS pid_trunk_fused_pitch_;
  control_toolbox::PidROS pid_trunk_fused_roll_;
  control_toolbox::PidROS pid_step_length_adjustment_pitch_;
  control_toolbox::PidROS pid_step_length_adjustment_roll_;
};
}  // namespace bitbots_quintic_walk

#endif  // BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_
