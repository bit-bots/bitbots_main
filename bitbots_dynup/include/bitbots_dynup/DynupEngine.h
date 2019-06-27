#ifndef BITBOTS_DYNUP_H
#define BITBOTS_DYNUP_H

#include <string>
#include <optional>
#include <bitbots_splines/SmoothSpline.hpp>
#include <bitbots_splines/SplineContainer.hpp>
#include <tf/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "Stabilizer.h"
#include <math.h>

typedef bitbots_splines::SplineContainer<bitbots_splines::SmoothSpline> Trajectories;

class DynUpParams {
public:
    double foot_rise;
    double foot_distance;
    double kick_windup_distance;

    double move_trunk_time = 1;
    double raise_foot_time = 1;
    double move_to_ball_time = 1;
    double kick_time = 1;
    double move_back_time = 1;
    double lower_foot_time = 1;
    double move_trunk_back_time = 1;

    double stabilizing_point_x;
    double stabilizing_point_y;

    double choose_foot_corridor_width;
};

/**
 * TODO
 */
class DynupEngine {
public:
    DynupEngine();


    /**
     * Do one iteration of spline-progress-updating. This means that whenever tick() is called,
     *      new position goals are retrieved from previously calculated splines, stabilized and transformed into
     *      JointGoals
     * @param dt Passed delta-time between last call to tick() and now. Measured in seconds
     * @return New motor goals only if a goal is currently set, position extractions from splines was possible and
     *      bio_ik was able to compute valid motor positions
     */
    std::optional<JointGoals> tick(double dt);

    void start(bool front);

    int get_percent_done() const;

    void set_params(DynUpParams params);

    Stabilizer m_stabilizer;
private:
    double m_time;
    std::optional<Trajectories> m_foot_trajectories, m_hand_trajectories;
    DynUpParams m_params;

    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_listener;
    geometry_msgs::PoseStamped  get_current_pose(Trajectories spline_container, std::string frame_id);

    void calc_front_splines(const geometry_msgs::Pose &foot_pose);
    void calc_back_splines();
    void calc_squat_splines();
    void init_trajectories();



};

#endif  // BITBOTS_DYNUP_H
