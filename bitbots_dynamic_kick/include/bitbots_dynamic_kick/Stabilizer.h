#ifndef BITBOTS_DYNAMIC_KICK_STABILIZER_H
#define BITBOTS_DYNAMIC_KICK_STABILIZER_H

#include <optional>
#include <geometry_msgs/Pose.h>

typedef std::map<std::string, double> JointGoals;

class Stabilizer {
public:
    Stabilizer();
    std::optional<JointGoals> stabilize(bool is_left_kick, geometry_msgs::Pose foot_goal);
};

#endif  // BITBOTS_DYNAMIC_KICK_STABILIZER_H
