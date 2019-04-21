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

    /**
     * Set new goal which the engine tries to kick at. This will remove the old goal completely and reset all splines.
     * @param target_pose Pose in base_link frame which should be reached.
     *      Ideally this is also where the ball lies and a kick occurs.
     * @param speed Speed with which to reach the target // TODO document scale of speed (m/s, km/h, 0-1,...)
     * @param l_foot_pose Current pose of left foot in base_link frame
     * @param r_foot_pose Current pos of right foot in base_link frame
     */
    void set_goal(const geometry_msgs::Pose& target_pose, double speed,
            const geometry_msgs::Pose& l_foot_pose, const geometry_msgs::Pose& r_foot_pose);

    /**
     * Reset this KickEngine completely, removing the goal, all splines and thereby stopping all output
     */
    void reset();

    /**
     * Do one iteration of spline-progress-updating. This means that whenever tick() is called,
     *      new motor goals are retrieved from previously calculated splines, stabilized and set to goals
     * @param dt Passed delta-time between last call to tick() and now. Measured in seconds
     * @param goals Output motor positions which need to be reached to advance the overall kick movement
     * @return Whether calculating new motor positions was successful or not.
     *      This could fail because no goal is set, more time has passed then the spline was long or bio_ik was not
     *      able to calculate motor_positions from splines.
     */
    bool tick(double dt, JointGoals& goals);

    /**
     * Is the currently performed kick with the left foor or not
     */
    bool is_left_kick();
private:
    geometry_msgs::Pose m_goal_pose;
    double m_speed;
    double m_time;
    Trajectories m_trajectories;
    Stabilizer m_stabilizer;

    /**
     * Construct m_trajectories and add all required splines with their respective keys
     */
    void init_trajectories();

    // TODO Add docstring to calc_splines once we resolve issue #2
    void calc_splines(const geometry_msgs::Pose& target_pose, const geometry_msgs::Pose& r_foot_pose);
};

#endif  // BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H
