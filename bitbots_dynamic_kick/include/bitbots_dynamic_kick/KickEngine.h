#ifndef BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H
#define BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H

#include <optional>
#include <std_msgs/Header.h>
#include <bitbots_splines/SmoothSpline.hpp>
#include <bitbots_splines/SplineContainer.hpp>
#include <bitbots_msgs/KickGoal.h>
#include <bitbots_msgs/KickFeedback.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include "Stabilizer.h"
#include "Visualizer.h"
#include <math.h>

typedef bitbots_splines::SplineContainer<bitbots_splines::SmoothSpline> Trajectories;

class KickParams {
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


enum KickPhase {
    Initial,
    MoveTrunk,
    RaiseFoot,
    Windup,
    Kick,
    MoveBack,
    LowerFoot,
    MoveTrunkBack,
    Done
};

/**
 * An instance of this class describes after which time a KickPhase is done
 */
class PhaseTimings {
public:
    double move_trunk;
    double raise_foot;
    double windup;
    double kick;
    double move_back;
    double lower_foot;
    double move_trunk_back;
};


/**
 * The KickEngine takes care of choosing an optimal foot to reach a given goal,
 * planning that foots required movement (rotation and positioning)
 * and updating short-term MotorGoals to move (the foot) along that planned path.
 *
 * It is vital to call the engines tick() method repeatedly because that is where these short-term MotorGoals are
 * returned.
 *
 * The KickEngine utilizes a Stabilizer to balance the robot during foot movments.
 */
class KickEngine {
public:
    explicit KickEngine();

    /**
     * Set new goal which the engine tries to kick at. This will remove the old goal completely and plan new splines.
     * @param header Definition of frame and time in which the goals were published
     * @param ball_position Position of the ball
     * @param kick_direction Direction into which to kick the ball
     * @param kick_speed Speed with which to kick the ball
     * @param r_foot_pose Current pose of right foot in l_sole frame
     * @param l_foot_pose Current pose of left foot in r_sole frame
     */
    bool set_goal(const std_msgs::Header &header,
                  const geometry_msgs::Vector3 &ball_position,
                  const geometry_msgs::Quaternion &kick_direction,
                  const float kick_speed,
                  const geometry_msgs::Pose &r_foot_pose,
                  const geometry_msgs::Pose &l_foot_pose);

    /**
     * Reset this KickEngine completely, removing the goal, all splines and thereby stopping all output
     */
    void reset();

    /**
     * Do one iteration of spline-progress-updating. This means that whenever tick() is called,
     *      new position goals are retrieved from previously calculated splines, stabilized and transformed into
     *      JointGoals
     * @param dt Passed delta-time between last call to tick() and now. Measured in seconds
     * @return New motor goals only if a goal is currently set, position extractions from splines was possible and
     *      bio_ik was able to compute valid motor positions
     */
    std::optional<JointGoals> tick(double dt);

    /**
     * Transform then goal into our support_foots frame
     * @param support_foot_frame Name of the support foots frame, meaning where to transform to
     * @param header Frame and time in which the goals were published
     * @param ball_position Position of the ball
     * @param kick_direction Direction in which to kick the ball
     * @return pair of (transformed_pose, transformed_direction)
     */
    std::optional<std::pair<geometry_msgs::Point, geometry_msgs::Quaternion>> transform_goal(
            const std::string &support_foot_frame,
            const std_msgs::Header &header,
            const geometry_msgs::Vector3 &ball_position,
            const geometry_msgs::Quaternion &kick_direction);

    /**
     * Is the currently performed kick with the left foot or not
     */
    bool is_left_kick();

    int get_percent_done() const;

    void set_params(KickParams params);

    const KickPhase getPhase();

    Stabilizer m_stabilizer;
private:
    double m_time;
    tf2::Vector3 m_ball_position;
    tf2::Quaternion m_kick_direction;
    float m_kick_speed;
    bool m_is_left_kick;
    std::optional<Trajectories> m_support_point_trajectories, m_flying_trajectories;
    KickParams m_params;
    PhaseTimings m_phase_timings;

    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_listener;

    /**
     * Construct m_trajectories and add all required splines with their respective keys
     */
    void init_trajectories();

    /**
     *  Calculate splines for a complete kick whereby m_is_left_kick should already be set correctly
     *
     *  @param flying_foot_pose Current pose of the foot which is supposed to be the flying/kicking one
     */
    void calc_splines(const geometry_msgs::Pose &flying_foot_pose);


    /**
     *  Calculate the point from which to perform the final kicking movement
     */
    tf2::Vector3 calc_kick_windup_point();

    /**
     * Choose with which foot the kick should be performed
     *
     * This is done by checking whether the ball is outside of a corridor ranging from base_footprint forward.
     * If it is, the foot on that side will be chosen as the kicking foot.
     * If not, a more fine grained angle based criterion is used.     *
     *
     * @param header Definition of frame and time in which the goals were published
     * @param ball_position Position where the ball is currently located
     * @param kick_movement Direction into which the ball should be kicked
     * @return Whether the resulting kick should be performed with the left foot
     */
    bool calc_is_left_foot_kicking(const std_msgs::Header &header,
                                   const geometry_msgs::Vector3 &ball_position,
                                   const geometry_msgs::Quaternion &kick_direction);

    geometry_msgs::PoseStamped get_current_pose(Trajectories spline_container);

    /**
     * Calculate the yaw of the kicking foot, so that it is turned
     * in the direction of the kick
     */
    double calc_kick_foot_yaw();
};

#endif  // BITBOTS_DYNAMIC_KICK_KICK_ENGINE_H
