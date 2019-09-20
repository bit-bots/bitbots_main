#include "bitbots_quintic_walk/walk_stabilizer.h"

namespace bitbots_quintic_walk {

WalkStabilizer::WalkStabilizer() {
  reset();
}

void WalkStabilizer::reset() {
}

std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> WalkStabilizer::stabilize(const WalkResponse &response) {
  auto ik_options = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
  ik_options->replace = true;
  ik_options->return_approximate_solution = true;

  // change goals from support foot based coordinate system to trunk based coordinate system
  tf2::Transform trunk_to_support_foot_goal = response.support_foot_to_trunk.inverse();
  tf2::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal*response.support_foot_to_flying_foot;

  //todo add pose goals, code is in old bioiksolver
  return std::move(ik_options);
}
}