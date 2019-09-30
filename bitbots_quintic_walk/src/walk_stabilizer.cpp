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

  // trunk goal
  auto *trunk_goal = new ReferencePoseGoal();
  trunk_goal->setPosition(response.support_foot_to_trunk.getOrigin());
  trunk_goal->setOrientation(response.support_foot_to_trunk.getRotation());
  trunk_goal->setLinkName("base_link");
  if (response.is_left_support_foot) {
    trunk_goal->setReferenceLinkName("l_sole");
  } else {
    trunk_goal->setReferenceLinkName("r_sole");
  }
  trunk_goal->setWeight(1);
  ik_options->goals.emplace_back(trunk_goal);

  // flying foot goal
  auto *fly_goal = new ReferencePoseGoal();
  fly_goal->setPosition(response.support_foot_to_flying_foot.getOrigin());
  fly_goal->setOrientation(response.support_foot_to_flying_foot.getRotation());
  if (response.is_left_support_foot) {
    fly_goal->setLinkName("r_sole");
    fly_goal->setReferenceLinkName("l_sole");
  } else {
    fly_goal->setLinkName("l_sole");
    fly_goal->setReferenceLinkName("r_sole");
  }
  fly_goal->setWeight(1);
  ik_options->goals.emplace_back(fly_goal);

  return std::move(ik_options);
}
}