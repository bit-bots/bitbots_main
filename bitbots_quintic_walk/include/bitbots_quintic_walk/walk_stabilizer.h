#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_

#include "bitbots_splines/abstract_stabilizer.h"

#include <optional>
#include <control_toolbox/pid.h>
#include <bitbots_splines/abstract_stabilizer.h>
#include "bitbots_quintic_walk/walk_utils.h"
#include <rot_conv/rot_conv.h>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

namespace bitbots_quintic_walk {

class WalkStabilizer : public bitbots_splines::AbstractStabilizer<WalkResponse> {
 public:
  explicit WalkStabilizer(const std::string ns);
  void reset() override;
  WalkResponse stabilize(const WalkResponse &response, const ros::Duration &dt) override;

 private:
  control_toolbox::Pid pid_trunk_fused_pitch_;
  control_toolbox::Pid pid_trunk_fused_roll_;
};
} // namespace bitbots_quintic_walk

#endif //BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_
