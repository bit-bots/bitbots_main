#ifndef BITBOTS_DYNAMIC_KICK_STABILIZER_H
#define BITBOTS_DYNAMIC_KICK_STABILIZER_H

#include <geometry_msgs/Pose.h>

typedef std::map<std::string, double> JointGoals;

class Stabilizer {
public:
    Stabilizer();
    bool stabilize(bool is_left_kick, geometry_msgs::Pose foot_goal, JointGoals& goals);
};

#endif  // BITBOTS_DYNAMIC_KICK_STABILIZER_H
