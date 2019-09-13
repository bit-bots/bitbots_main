#ifndef BITBOTS_SPLINES_ABSTRACTIK_H
#define BITBOTS_SPLINES_ABSTRACTIK_H

#include <vector>
#include <string>
#include <bio_ik/bio_ik.h>

typedef std::pair<std::vector<std::string>, std::vector<double>> JointGoals;

class AbstractIK {
    virtual JointGoals calculate(std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) = 0;
    virtual void init(moveit::core::RobotModelPtr kinematic_model) = 0;
    virtual void reset() = 0;
};

#endif //BITBOTS_SPLINES_ABSTRACTIK_H
