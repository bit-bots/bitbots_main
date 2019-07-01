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
    double leg_min_length;
    double arm_max_length;
    double time_foot_close;
    double time_hands_front;
    double time_foot_ground;
    double time_torso_45;

    double foot_distance;
    double rise_time;
    double trunk_height;
    double trunk_pitch;
    double start_x;
    double start_trunk_height;
    double start_pitch;

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

    void reset();

    Stabilizer m_stabilizer;
private:
    double m_time;

    std::optional<Trajectories> m_foot_trajectories, m_hand_trajectories;
    DynUpParams m_params;

    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_listener;
    geometry_msgs::PoseStamped  get_current_foot_pose(Trajectories spline_container,  bool left_foot);

    void calc_front_splines();
    void calc_back_splines();
    void calc_squat_splines();
    void init_trajectories();



};

#endif  // BITBOTS_DYNUP_H
