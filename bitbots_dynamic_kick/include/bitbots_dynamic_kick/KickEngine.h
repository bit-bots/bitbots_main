#ifndef BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H
#define BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H

#include <bitbots_splines/SmoothSpline.hpp>
#include <bitbots_splines/SplineContainer.hpp>
#include <bitbots_msgs/KickGoal.h>
#include <bitbots_msgs/KickFeedback.h>
#include <tf/LinearMath/Transform.h>
#include "Stabilizer.h"

typedef bitbots_splines::SplineContainer<bitbots_splines::SmoothSpline> Trajectories;

class KickEngine {
public:
    KickEngine();
    void set_goal(const geometry_msgs::Pose& target_pose, double speed,
            const geometry_msgs::Pose& l_foot_pose, const geometry_msgs::Pose& r_foot_pose);
    void reset();
    bool tick(double dt, JointGoals& goals);
    bool is_left_kick();
private:
    geometry_msgs::Pose m_goal_pose;
    double m_speed;
    double m_time;
    Trajectories m_trajectories;
    Stabilizer m_stabilizer;

    void init_trajectories();
    void calc_splines(const geometry_msgs::Pose& target_pose, const geometry_msgs::Pose& r_foot_pose);
};

#endif  // BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H
