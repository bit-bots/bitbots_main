#include "bitbots_dynamic_kick/KickEngine.h"

KickEngine::KickEngine() :
        m_listener(m_tf_buffer)  {
}

void KickEngine::reset() {
    m_time = 0;
    m_support_point_trajectories.reset();
    m_flying_trajectories.reset();
}

bool KickEngine::set_goal(const std_msgs::Header &header,
                          const geometry_msgs::Vector3 &ball_position,
                          const geometry_msgs::Quaternion &kick_direction,
                          const float kick_speed,
                          const geometry_msgs::Pose &r_foot_pose,
                          const geometry_msgs::Pose &l_foot_pose) {

    m_is_left_kick = calc_is_left_foot_kicking(header, ball_position, kick_direction); // TODO Internal state is dirty when goal transformation fails

    /* Save given goals because we reuse them later */
    auto transformed_goal = transform_goal((m_is_left_kick) ? "r_sole" : "l_sole", header, ball_position,
                                           kick_direction);
    if (transformed_goal) {
        m_stabilizer.reset();
        tf2::convert(transformed_goal->first, m_ball_position);
        tf2::convert(transformed_goal->second, m_kick_direction);
        m_kick_direction.normalize();
        m_kick_speed = kick_speed;

        m_time = 0;

        /* Plan new splines according to new goal */
        init_trajectories();
        calc_splines(m_is_left_kick ? l_foot_pose : r_foot_pose);
        m_stabilizer.m_visualizer.display_flying_splines(m_flying_trajectories.value(), (m_is_left_kick) ? "r_sole" : "l_sole");

        return true;

    } else {
        return false;
    }
}

std::optional<JointGoals> KickEngine::tick(double dt) {
    /* Only do an actual tick when splines are present */
    if (m_support_point_trajectories && m_flying_trajectories) {
        /* Get should-be pose from planned splines (every axis) at current time */
        geometry_msgs::Point support_point;
        support_point.x = m_support_point_trajectories.value().get("pos_x").pos(m_time);
        support_point.y = m_support_point_trajectories.value().get("pos_y").pos(m_time);
        geometry_msgs::PoseStamped flying_foot_pose = get_current_pose(m_flying_trajectories.value());

        /* calculate if we want to use center-of-pressure in the current phase */
        bool cop_support_point;
        /* use COP based support point only when the weight is on the support foot
         * while raising/lowering the foot, the weight is not completely on the support foot (that's why /2.0)*/
        if (m_time > m_params.move_trunk_time + m_params.raise_foot_time / 2.0 &&
            m_time < m_phase_timings.move_back + m_params.lower_foot_time / 2.0) {
            cop_support_point = true;
        } else {
            cop_support_point = false;
        }

        m_time += dt;

        /* Stabilize and return result */
        return m_stabilizer.stabilize(m_is_left_kick, support_point, flying_foot_pose, cop_support_point);
    } else {
        return std::nullopt;
    }
}

geometry_msgs::PoseStamped KickEngine::get_current_pose(Trajectories spline_container) {
    geometry_msgs::PoseStamped foot_pose;
    foot_pose.header.frame_id = "l_sole";
    foot_pose.header.stamp = ros::Time::now();
    foot_pose.pose.position.x = spline_container.get("pos_x").pos(m_time);
    foot_pose.pose.position.y = spline_container.get("pos_y").pos(m_time);
    foot_pose.pose.position.z = spline_container.get("pos_z").pos(m_time);
    tf2::Quaternion q;
    /* Apparently, the axis order is different than expected */
    q.setEuler(spline_container.get("pitch").pos(m_time),
               spline_container.get("roll").pos(m_time),
               spline_container.get("yaw").pos(m_time));
    foot_pose.pose.orientation.x = q.x();
    foot_pose.pose.orientation.y = q.y();
    foot_pose.pose.orientation.z = q.z();
    foot_pose.pose.orientation.w = q.w();
    return foot_pose;
}

void KickEngine::calc_splines(const geometry_msgs::Pose &flying_foot_pose) {
    /*
     * Add current position, target position and current position to splines so that they describe a smooth
     * curve to the ball and back
     */
    /* Splines:
     * - stand
     * - move trunk
     * - raise foot
     * - kick
     * - move foot back
     * - lower foot
     *  - move trunk back
     */

    /* calculate timings for this kick */
    m_phase_timings.move_trunk = 0 + m_params.move_trunk_time;
    m_phase_timings.raise_foot = m_phase_timings.move_trunk + m_params.raise_foot_time;
    m_phase_timings.windup = m_phase_timings.raise_foot + m_params.move_to_ball_time;
    m_phase_timings.kick = m_phase_timings.windup + m_params.kick_time;
    m_phase_timings.move_back = m_phase_timings.kick + m_params.move_back_time;
    m_phase_timings.lower_foot = m_phase_timings.move_back + m_params.lower_foot_time;
    m_phase_timings.move_trunk_back = m_phase_timings.lower_foot + m_params.move_trunk_back_time;

    int kick_foot_sign;
    if (m_is_left_kick) {
        kick_foot_sign = 1;
    } else {
        kick_foot_sign = -1;
    }

    tf2::Vector3 kick_windup_point = calc_kick_windup_point();

    /* Flying foot position */
    m_flying_trajectories->get("pos_x").addPoint(0, flying_foot_pose.position.x);
    m_flying_trajectories->get("pos_x").addPoint(m_phase_timings.move_trunk, 0);
    m_flying_trajectories->get("pos_x").addPoint(m_phase_timings.raise_foot, 0);
    m_flying_trajectories->get("pos_x").addPoint(m_phase_timings.windup, kick_windup_point.x(), 0, 0);
    m_flying_trajectories->get("pos_x").addPoint(m_phase_timings.kick, m_ball_position.x(),
                                                 m_kick_direction.x() * m_kick_speed, 0);
    m_flying_trajectories->get("pos_x").addPoint(m_phase_timings.move_back, 0);
    m_flying_trajectories->get("pos_x").addPoint(m_phase_timings.lower_foot, 0);
    m_flying_trajectories->get("pos_x").addPoint(m_phase_timings.move_trunk_back, 0);

    m_flying_trajectories->get("pos_y").addPoint(0, flying_foot_pose.position.y);
    m_flying_trajectories->get("pos_y").addPoint(m_phase_timings.move_trunk, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(m_phase_timings.raise_foot, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(m_phase_timings.windup, kick_windup_point.y(), 0, 0);
    m_flying_trajectories->get("pos_y").addPoint(m_phase_timings.kick, m_ball_position.y(), m_kick_direction.y() * m_kick_speed, 0);
    m_flying_trajectories->get("pos_y").addPoint(m_phase_timings.move_back, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(m_phase_timings.lower_foot, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(m_phase_timings.move_trunk_back, kick_foot_sign * m_params.foot_distance);

    m_flying_trajectories->get("pos_z").addPoint(0, flying_foot_pose.position.z);
    m_flying_trajectories->get("pos_z").addPoint(m_phase_timings.move_trunk, 0);
    m_flying_trajectories->get("pos_z").addPoint(m_phase_timings.raise_foot, m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(m_phase_timings.windup, m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(m_phase_timings.kick, m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(m_phase_timings.move_back, m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(m_phase_timings.lower_foot, 0.4 * m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(m_phase_timings.move_trunk_back, 0);

    /* Flying foot orientation */
    /* Construct a start_rotation as quaternion from Pose msg */
    tf::Quaternion start_rotation(flying_foot_pose.orientation.x, flying_foot_pose.orientation.y,
                                  flying_foot_pose.orientation.z, flying_foot_pose.orientation.w);
    double start_r, start_p, start_y;
    tf::Matrix3x3(start_rotation).getRPY(start_r, start_p, start_y);

    /* Also construct one for the target */
    tf::Quaternion target_rotation(flying_foot_pose.orientation.x, flying_foot_pose.orientation.y,
                                   flying_foot_pose.orientation.z, flying_foot_pose.orientation.w);
    double target_r, target_p, target_y;
    tf::Matrix3x3(target_rotation).getRPY(target_r, target_p, target_y);

    target_y = calc_kick_foot_yaw();

    /* Add these quaternions in the same fashion as before to our splines (current, target, current) */
    m_flying_trajectories->get("roll").addPoint(0, start_r);
    m_flying_trajectories->get("roll").addPoint(m_phase_timings.windup, start_r);
    m_flying_trajectories->get("roll").addPoint(m_phase_timings.move_trunk_back, start_r);
    m_flying_trajectories->get("pitch").addPoint(0, start_p);
    m_flying_trajectories->get("pitch").addPoint(m_phase_timings.windup, start_p);
    m_flying_trajectories->get("pitch").addPoint(m_phase_timings.move_trunk_back, start_p);
    m_flying_trajectories->get("yaw").addPoint(0, start_y);
    m_flying_trajectories->get("yaw").addPoint(m_phase_timings.raise_foot, start_y);
    m_flying_trajectories->get("yaw").addPoint(m_phase_timings.windup, target_y);
    m_flying_trajectories->get("yaw").addPoint(m_phase_timings.kick, target_y);
    m_flying_trajectories->get("yaw").addPoint(m_phase_timings.move_back, start_y);
    m_flying_trajectories->get("yaw").addPoint(m_phase_timings.move_trunk_back, start_y);

    /* Stabilizing point */
    m_support_point_trajectories->get("pos_x").addPoint(0, 0);
    m_support_point_trajectories->get("pos_x").addPoint(m_phase_timings.move_trunk, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(m_phase_timings.raise_foot, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(m_phase_timings.windup, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(m_phase_timings.kick, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(m_phase_timings.move_back, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(m_phase_timings.lower_foot, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(m_phase_timings.move_trunk_back, 0);

    m_support_point_trajectories->get("pos_y").addPoint(0, kick_foot_sign * (m_params.foot_distance / 2.0));
    m_support_point_trajectories->get("pos_y").addPoint(m_phase_timings.move_trunk, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(m_phase_timings.raise_foot, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(m_phase_timings.windup, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(m_phase_timings.kick, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(m_phase_timings.move_back, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(m_phase_timings.lower_foot, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(m_phase_timings.move_trunk_back, kick_foot_sign * (m_params.foot_distance / 2.0));
}

void KickEngine::init_trajectories() {
    m_support_point_trajectories = Trajectories();

    m_support_point_trajectories->add("pos_x");
    m_support_point_trajectories->add("pos_y");

    m_flying_trajectories = Trajectories();

    m_flying_trajectories->add("pos_x");
    m_flying_trajectories->add("pos_y");
    m_flying_trajectories->add("pos_z");

    m_flying_trajectories->add("roll");
    m_flying_trajectories->add("pitch");
    m_flying_trajectories->add("yaw");
}

std::optional<std::pair<geometry_msgs::Point, geometry_msgs::Quaternion>> KickEngine::transform_goal(
        const std::string &support_foot_frame,
        const std_msgs::Header &header,
        const geometry_msgs::Vector3 &ball_position,
        const geometry_msgs::Quaternion &kick_direction) {
    /* construct stamped goals so that they can be transformed */ // TODO Extract this into own function because we do it multiple times
    geometry_msgs::PointStamped stamped_position;       // TODO Make KickGoal a point as well so we dont have to do transformations here
    stamped_position.point.x = ball_position.x;
    stamped_position.point.y = ball_position.y;
    stamped_position.point.z = ball_position.z;
    stamped_position.header = header;
    //stamped_position.vector = ball_position;
    geometry_msgs::QuaternionStamped stamped_direction;
    stamped_direction.header = header;
    stamped_direction.quaternion = kick_direction;

    /* do transform into support_foot frame */
    geometry_msgs::PointStamped transformed_position;
    geometry_msgs::QuaternionStamped transformed_direction;

    m_tf_buffer.transform(stamped_position, transformed_position, support_foot_frame, ros::Duration(0.2));
    m_tf_buffer.transform(stamped_direction, transformed_direction, support_foot_frame, ros::Duration(0.2));

    auto x = m_tf_buffer.lookupTransform(support_foot_frame, header.frame_id, header.stamp, ros::Duration(0.2));

    return std::pair(transformed_position.point, transformed_direction.quaternion);
}

tf2::Vector3 KickEngine::calc_kick_windup_point() {
    /* retrieve yaw from m_kick_direction */
    double yaw = tf2::getYaw(m_kick_direction);

    /* create a vector which points in the negative direction of m_kick_direction */
    tf2::Vector3 vec(cos(yaw), sin(yaw), 0);
    vec.normalize();

    /* take windup distance into account */
    vec *= -m_params.kick_windup_distance;

    /* add the ball position because the windup point is in support_foot_frame and not ball_frame */
    vec += m_ball_position;

    vec.setZ(m_params.foot_rise);

    m_stabilizer.m_visualizer.display_windup_point(vec, (m_is_left_kick) ? "r_sole" : "l_sole");
    return vec;
}

bool KickEngine::calc_is_left_foot_kicking(const std_msgs::Header &header,
                                           const geometry_msgs::Vector3 &ball_position,
                                           const geometry_msgs::Quaternion &kick_direction) {
    /* prepare variables with stamps */
    geometry_msgs::Vector3Stamped stamped_position;
    stamped_position.header = header;
    stamped_position.vector = ball_position;
    geometry_msgs::QuaternionStamped stamped_direction;
    stamped_direction.header = header;
    stamped_direction.quaternion = kick_direction;

    /* transform ball data into frame where we want to apply it */
    tf2::Stamped<tf2::Vector3> transformed_ball_position;
    m_tf_buffer.transform(stamped_position, transformed_ball_position, "base_footprint", ros::Duration(0.2));
    tf2::Stamped<tf2::Quaternion> transformed_direction;
    m_tf_buffer.transform(stamped_direction, transformed_direction, "base_footprint", ros::Duration(0.2));

    /*
     * check if ball is outside of an imaginary corridor
     * if it is not, we use a more fined grained criterion which takes kick_direction into account
     */
    if (transformed_ball_position.y() > m_params.choose_foot_corridor_width / 2)
        return true;
    else if (transformed_ball_position.y() < -m_params.choose_foot_corridor_width / 2)
        return false;

    /* use the more fine grained angle based criterion
     * angle1 = angle between "forward" and "origin-to-ball-position"
     * angle2 = yaw of kick_direction
     * angle_diff = difference between the two on which the decision happens
     */
    double angle1 = transformed_ball_position.angle({1, 0, 0});
    angle1 *= transformed_ball_position.y() < 0 ? -1 : 1;

    double angle2 = tf2::getYaw(transformed_direction);
    double angle_diff = angle2 - angle1;

    ROS_INFO_STREAM("Choosing " << ((angle_diff < 0) ? "left" : "right") << " foot to kick");

    return angle_diff < 0;
}

double KickEngine::calc_kick_foot_yaw() {
    double kick_roll_angle, kick_pitch_angle, kick_yaw_angle;
    tf2::Matrix3x3(m_kick_direction).getRPY(kick_roll_angle, kick_pitch_angle, kick_yaw_angle);

    if (kick_yaw_angle > M_PI_4) {
        return kick_yaw_angle - M_PI_2;
    } else if (kick_yaw_angle < -M_PI_4) {
        return kick_yaw_angle + M_PI_2;
    } else {
        return kick_yaw_angle;
    }

}

bool KickEngine::is_left_kick() {
    return m_is_left_kick;
}

int KickEngine::get_percent_done() const {
    return int(m_time / m_phase_timings.move_trunk_back * 100);
}

const KickPhase KickEngine::getPhase() {
    if (m_time == 0)
        return KickPhase::Initial;
    else if (m_time <= m_phase_timings.move_trunk)
        return KickPhase::MoveTrunk;
    else if (m_time <= m_phase_timings.raise_foot)
        return KickPhase::RaiseFoot;
    else if (m_time <= m_phase_timings.windup)
        return KickPhase::Windup;
    else if (m_time <= m_phase_timings.kick)
        return KickPhase::Kick;
    else if (m_time <= m_phase_timings.move_back)
        return KickPhase::MoveBack;
    else if (m_time <= m_phase_timings.lower_foot)
        return KickPhase::LowerFoot;
    else if (m_time <= m_phase_timings.move_trunk_back)
        return KickPhase::MoveTrunkBack;
    else
        return KickPhase::Done;
}

void KickEngine::set_params(KickParams params) {
    m_params = params;
}
