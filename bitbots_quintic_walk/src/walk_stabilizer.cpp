#include "bitbots_quintic_walk/walk_stabilizer.h"

namespace bitbots_quintic_walk {

WalkStabilizer::WalkStabilizer() {
  reset();
}

void WalkStabilizer::reset() {
}

std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> WalkStabilizer::stabilize(const WalkResponse &response) {
  auto ik_options = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
  //todo this class currently not doing anything and has to be refactored after merging PID control
  return std::move(ik_options);
}
}