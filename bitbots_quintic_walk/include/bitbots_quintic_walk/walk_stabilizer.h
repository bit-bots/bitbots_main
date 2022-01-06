#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_

#include "bitbots_splines/abstract_stabilizer.h"

#include <optional>
#include <control_toolbox/pid_ros.hpp>
#include <bitbots_splines/abstract_stabilizer.h>
#include "bitbots_quintic_walk/walk_utils.h"
#include <rot_conv/rot_conv.h>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace bitbots_quintic_walk {

class WalkStabilizer : public bitbots_splines::AbstractStabilizer<WalkResponse> {
 public:
  explicit WalkStabilizer(rclcpp::Node::SharedPtr node);
  void reset() override;
  WalkResponse stabilize(const WalkResponse &response, const rclcpp::Duration &dt) override;

 private:
  std::shared_ptr<rclcpp::Node> node_;
  control_toolbox::PidROS pid_trunk_fused_pitch_;
  control_toolbox::PidROS pid_trunk_fused_roll_;
};
} // namespace bitbots_quintic_walk

#endif //BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_