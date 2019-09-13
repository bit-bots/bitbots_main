#ifndef BITBOTS_SPLINES_ABSTRACTSTABILIZER_H
#define BITBOTS_SPLINES_ABSTRACTSTABILIZER_H

#include <bio_ik/goal.h>

template<class Positions>
class AbstractStabilizer {
public:
    virtual void reset() = 0;
    virtual std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> stabilize(const Positions& positions) = 0;

};

#endif //BITBOTS_SPLINES_ABSTRACTSTABILIZER_H
