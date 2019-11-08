#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_

#include "bitbots_splines/abstract_stabilizer.h"

#include <optional>
#include <control_toolbox/pid.h>
#include <bio_ik/bio_ik.h>
#include <bitbots_splines/abstract_stabilizer.h>
#include "bitbots_quintic_walk/walk_utils.h"
#include "bitbots_splines/dynamic_balancing_goal.h"
#include "bitbots_splines/reference_goals.h"

namespace bitbots_quintic_walk {

class WalkStabilizer : public bitbots_splines::AbstractStabilizer<WalkResponse> {
 public:
  WalkStabilizer();
  virtual void reset();
  std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> stabilize(const WalkResponse &response, double current_pitch, const ros::Duration &dt);

 private:
   control_toolbox::Pid pid_trunk_pitch_;
};
}

#endif //BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_STABILIZER_H_
