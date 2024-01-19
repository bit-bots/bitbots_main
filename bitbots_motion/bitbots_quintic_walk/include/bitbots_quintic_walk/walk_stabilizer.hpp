#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_

#include <bitbots_splines/abstract_stabilizer.h>
#include <rot_conv/rot_conv.h>

#include <Eigen/Geometry>
#include <control_toolbox/pid_ros.hpp>
#include <optional>
#include <rclcpp/logger.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "bitbots_quintic_walk/walk_utils.h"
#include "bitbots_splines/abstract_stabilizer.h"

namespace bitbots_quintic_walk {

class WalkStabilizer : public bitbots_splines::AbstractStabilizer<WalkResponse> {
 public:
  explicit WalkStabilizer(std::string ns);
  void reset() override;
  WalkResponse stabilize(const WalkResponse &response, const rclcpp::Duration &dt) override;

 private:
  std::shared_ptr<control_toolbox::PidROS> pid_trunk_fused_pitch_;
  std::shared_ptr<control_toolbox::PidROS> pid_trunk_fused_roll_;
  std::shared_ptr<rclcpp::Node> pitch_node_;
  std::shared_ptr<rclcpp::Node> roll_node_;
};
}  // namespace bitbots_quintic_walk

#endif  // BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_