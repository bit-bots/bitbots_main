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
  tf2::Transform trunk_to_support_foot = response.support_foot_to_trunk.inverse();
  tf2::Transform trunk_to_flying_foot = trunk_to_support_foot*response.support_foot_to_flying_foot;

  // trunk goal
  auto* support_goal = new bio_ik::PoseGoal();
  support_goal->setPosition(trunk_to_support_foot.getOrigin());
  support_goal->setOrientation(trunk_to_support_foot.getRotation());
  if(response.is_left_support_foot) {
    support_goal->setLinkName("l_sole");
  }else{
    support_goal->setLinkName("r_sole");
  }
  ik_options->goals.emplace_back(support_goal);

  // flying foot goal
  auto* fly_goal = new bio_ik::PoseGoal();
  fly_goal->setPosition(trunk_to_flying_foot.getOrigin());
  fly_goal->setOrientation(trunk_to_flying_foot.getRotation());
  if(!response.is_left_support_foot) {
    fly_goal->setLinkName("l_sole");
  }else{
    fly_goal->setLinkName("r_sole");
  }
  ik_options->goals.emplace_back(fly_goal);

  return std::move(ik_options);
}
}