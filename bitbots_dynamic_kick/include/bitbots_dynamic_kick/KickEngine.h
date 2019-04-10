#ifndef BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H
#define BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H

#include <bitbots_splines/TrajectoryUtils.h>
#include <bitbots_msgs/KickGoal.h>
#include <bitbots_msgs/KickFeedback.h>

class KickEngine {
public:
    KickEngine();
    void set_goal(const bitbots_msgs::KickGoalConstPtr& goal);
    void reset();
    bitbots_msgs::KickFeedback tick();
private:
    bitbots_msgs::KickGoal m_current_goal;
    bitbots_splines::Trajectories m_trajectories;
    void calc_splines();
};

#endif  // BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H
