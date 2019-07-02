#include "bitbots_dynamic_kick/KickEngine.h"

KickEngine::KickEngine(Visualizer &visualizer) :
        m_listener(m_tf_buffer),
        m_visualizer(visualizer){}

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

    m_is_left_kick = calc_is_left_foot_kicking(header, ball_position, kick_direction);

    /* Save given goals because we reuse them later */
    auto transformed_goal = transform_goal((m_is_left_kick) ? "r_sole" : "l_sole", header, ball_position,
                                           kick_direction);
    if (transformed_goal) {
        m_stabilizer.reset();
        tf2::convert(transformed_goal->first, m_ball_position);
        tf2::convert(transformed_goal->second, m_kick_direction);
        m_kick_speed = kick_speed;
        m_time = 0;

        /* Plan new splines according to new goal */
        init_trajectories();
        calc_splines(m_is_left_kick ? l_foot_pose : r_foot_pose);
        m_visualizer.display_flying_splines(m_flying_trajectories.value(), (m_is_left_kick) ? "r_sole" : "l_sole");

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

        m_time += dt;

        /* Stabilize and return result */
        return m_stabilizer.stabilize(m_is_left_kick, support_point, flying_foot_pose);
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
     * - lower foot and move trunk
     */
    /* The fix* variables describe the discrete points in time where the positions are given by the parameters.
     * Between them, the spline interpolation happens. */
    double fix0 = 0;
    double fix1 = fix0 + m_params.move_trunk_time;
    double fix2 = fix1 + m_params.raise_foot_time;
    double fix3 = fix2 + m_params.move_to_ball_time;
    double fix4 = fix3 + m_params.kick_time;
    double fix5 = fix4 + m_params.move_back_time;
    double fix6 = fix5 + m_params.lower_foot_time;
    double fix7 = fix6 + m_params.move_trunk_back_time;

    int kick_foot_sign;
    if (m_is_left_kick) {
        kick_foot_sign = 1;
    } else {
        kick_foot_sign = -1;
    }

    tf2::Vector3 kick_windup_point = calc_kick_windup_point();

    /* Flying foot position */
    m_flying_trajectories->get("pos_x").addPoint(fix0, flying_foot_pose.position.x);
    m_flying_trajectories->get("pos_x").addPoint(fix1, 0);
    m_flying_trajectories->get("pos_x").addPoint(fix2, 0);
    m_flying_trajectories->get("pos_x").addPoint(fix3, kick_windup_point.x(), 0, 0);
    m_flying_trajectories->get("pos_x").addPoint(fix4, m_ball_position.x(), m_kick_direction.x());
    m_flying_trajectories->get("pos_x").addPoint(fix5, 0);
    m_flying_trajectories->get("pos_x").addPoint(fix6, 0);
    m_flying_trajectories->get("pos_x").addPoint(fix7, 0);

    m_flying_trajectories->get("pos_y").addPoint(fix0, flying_foot_pose.position.y);
    m_flying_trajectories->get("pos_y").addPoint(fix1, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(fix2, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(fix3, kick_windup_point.y(), 0, 0);
    m_flying_trajectories->get("pos_y").addPoint(fix4, m_ball_position.y(), m_kick_speed, 0);
    m_flying_trajectories->get("pos_y").addPoint(fix5, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(fix6, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(fix7, kick_foot_sign * m_params.foot_distance);

    m_flying_trajectories->get("pos_z").addPoint(fix0, flying_foot_pose.position.z);
    m_flying_trajectories->get("pos_z").addPoint(fix1, 0);
    m_flying_trajectories->get("pos_z").addPoint(fix2, m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(fix3, m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(fix4, m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(fix5, m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(fix6, 0);
    m_flying_trajectories->get("pos_z").addPoint(fix7, 0);

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
    m_flying_trajectories->get("roll").addPoint(fix0, start_r);
    m_flying_trajectories->get("roll").addPoint(fix3, start_r);
    m_flying_trajectories->get("roll").addPoint(fix7, start_r);
    m_flying_trajectories->get("pitch").addPoint(fix0, start_p);
    m_flying_trajectories->get("pitch").addPoint(fix3, start_p);
    m_flying_trajectories->get("pitch").addPoint(fix7, start_p);
    m_flying_trajectories->get("yaw").addPoint(fix0, start_y);
    m_flying_trajectories->get("yaw").addPoint(fix3, target_y);
    m_flying_trajectories->get("yaw").addPoint(fix7, start_y);

    /* Stabilizing point */
    m_support_point_trajectories->get("pos_x").addPoint(fix0, 0);
    m_support_point_trajectories->get("pos_x").addPoint(fix1, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(fix2, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(fix3, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(fix4, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(fix5, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(fix6, m_params.stabilizing_point_x);
    m_support_point_trajectories->get("pos_x").addPoint(fix7, 0);

    m_support_point_trajectories->get("pos_y").addPoint(fix0, kick_foot_sign * (m_params.foot_distance / 2.0));
    m_support_point_trajectories->get("pos_y").addPoint(fix1, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(fix2, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(fix3, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(fix4, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(fix5, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(fix6, kick_foot_sign * (-m_params.stabilizing_point_y));
    m_support_point_trajectories->get("pos_y").addPoint(fix7, kick_foot_sign * (m_params.foot_distance / 2.0));
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

    m_visualizer.display_windup_point(vec, (m_is_left_kick) ? "r_sole" : "l_sole");
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
    double duration = m_params.move_trunk_time + m_params.raise_foot_time + m_params.move_to_ball_time +
                      m_params.kick_time + m_params.move_back_time + m_params.move_trunk_back_time +
                      m_params.lower_foot_time;
    return int(m_time / duration * 100);
}

void KickEngine::set_params(KickParams params) {
    m_params = params;
}
