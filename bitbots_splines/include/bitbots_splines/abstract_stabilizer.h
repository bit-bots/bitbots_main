#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACTSTABILIZER_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACTSTABILIZER_H_

#include <bio_ik/goal.h>

namespace bitbots_splines {

template<typename Positions>
class AbstractStabilizer {
 public:
  virtual void reset() = 0;
  virtual std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> stabilize(const Positions &positions) = 0;

};
}

#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACTSTABILIZER_H_
