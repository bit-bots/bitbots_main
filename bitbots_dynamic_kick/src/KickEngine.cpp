#include "bitbots_dynamic_kick/KickEngine.h"

KickEngine::KickEngine() {}

void KickEngine::reset() {
    m_time = 0;
    m_trunk_trajectories.reset();
    m_flying_trajectories.reset();
}

void KickEngine::set_goal(const geometry_msgs::Pose &target_pose, tf2::Vector3 speed,
                          const geometry_msgs::Pose &trunk_pose, const geometry_msgs::Pose &r_foot_pose,
                          bool is_left_kick) {
    /* Save given goals because we reuse them later */
    m_goal_pose = target_pose;
    m_speed = speed;
    m_time = 0;
    m_is_left_kick = is_left_kick;

    /* Plan new splines according to new goal */
    init_trajectories();
    calc_splines(target_pose, r_foot_pose, trunk_pose);
}

std::optional<JointGoals> KickEngine::tick(double dt) {
    /* Only do an actual tick when splines are present */
    if (m_trunk_trajectories && m_flying_trajectories) {
        /* Get should-be pose from planned splines (every axis) at current time */
        geometry_msgs::PoseStamped trunk_pose = get_current_pose(m_trunk_trajectories.value());
        geometry_msgs::PoseStamped flying_foot_pose = get_current_pose(m_flying_trajectories.value());

        m_time += dt;

        /* Stabilize and return result */
        return m_stabilizer.stabilize(/* is_left_kick */ m_is_left_kick, trunk_pose, flying_foot_pose);
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

void KickEngine::calc_splines(const geometry_msgs::Pose &target_pose, const geometry_msgs::Pose &flying_foot_pose,
                              const geometry_msgs::Pose &trunk_pose) {
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
    double fix3 = fix2 + m_params.kick_time;
    double fix4 = fix3 + m_params.move_back_time;
    double fix5 = fix4 + m_params.lower_foot_time;

    int kick_foot_sign;
    if(m_is_left_kick){
        kick_foot_sign = 1;
    }
    else
    {
        kick_foot_sign = -1;
    }


    /* Flying foot position */
    m_flying_trajectories->get("pos_x").addPoint(fix0, 0);
    m_flying_trajectories->get("pos_x").addPoint(fix1, 0);
    m_flying_trajectories->get("pos_x").addPoint(fix2, 0);
    m_flying_trajectories->get("pos_x").addPoint(fix3, m_params.kick_distance);
    m_flying_trajectories->get("pos_x").addPoint(fix4, 0);
    m_flying_trajectories->get("pos_x").addPoint(fix5, 0);

    // TODO adjust when the flying foot changes
    m_flying_trajectories->get("pos_y").addPoint(fix0, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(fix1, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(fix2, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(fix3, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(fix4, kick_foot_sign * m_params.foot_distance);
    m_flying_trajectories->get("pos_y").addPoint(fix5, kick_foot_sign * m_params.foot_distance);

    m_flying_trajectories->get("pos_z").addPoint(fix0, 0);
    m_flying_trajectories->get("pos_z").addPoint(fix1, 0);
    m_flying_trajectories->get("pos_z").addPoint(fix2, m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(fix3, m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(fix4, m_params.foot_rise);
    m_flying_trajectories->get("pos_z").addPoint(fix5, 0);

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

    /* Add these quaternions in the same fashion as before to our splines (current, target, current) */
    m_flying_trajectories->get("roll").addPoint(fix0, start_r);
    m_flying_trajectories->get("roll").addPoint(fix5, start_r);
    m_flying_trajectories->get("pitch").addPoint(fix0, start_p);
    m_flying_trajectories->get("pitch").addPoint(fix5, start_p);
    m_flying_trajectories->get("yaw").addPoint(fix0, start_y);
    m_flying_trajectories->get("yaw").addPoint(fix5, start_y);

    /* Trunk position */
    m_trunk_trajectories->get("pos_x").addPoint(fix0, 0);
    m_trunk_trajectories->get("pos_x").addPoint(fix1, 0);
    m_trunk_trajectories->get("pos_x").addPoint(fix2, 0);
    m_trunk_trajectories->get("pos_x").addPoint(fix3, 0);
    m_trunk_trajectories->get("pos_x").addPoint(fix4, 0);
    m_trunk_trajectories->get("pos_x").addPoint(fix5, 0);

    m_trunk_trajectories->get("pos_y").addPoint(fix0, kick_foot_sign * (m_params.foot_distance / 2.0));
    m_trunk_trajectories->get("pos_y").addPoint(fix1, kick_foot_sign * (m_params.foot_distance / 2.0 - m_params.trunk_movement));
    m_trunk_trajectories->get("pos_y").addPoint(fix2, kick_foot_sign * (m_params.foot_distance / 2.0 - m_params.trunk_movement - m_params.foot_rise_trunk_movement));
    m_trunk_trajectories->get("pos_y").addPoint(fix3, kick_foot_sign * (m_params.foot_distance / 2.0 - m_params.trunk_movement - m_params.foot_rise_trunk_movement));
    m_trunk_trajectories->get("pos_y").addPoint(fix4, kick_foot_sign * (m_params.foot_distance / 2.0 - m_params.trunk_movement - m_params.foot_rise_trunk_movement));
    m_trunk_trajectories->get("pos_y").addPoint(fix5, kick_foot_sign * (m_params.foot_distance / 2.0));

    m_trunk_trajectories->get("pos_z").addPoint(fix0, m_params.trunk_height);
    m_trunk_trajectories->get("pos_z").addPoint(fix1, m_params.trunk_height);
    m_trunk_trajectories->get("pos_z").addPoint(fix2, m_params.trunk_height);
    m_trunk_trajectories->get("pos_z").addPoint(fix3, m_params.trunk_height);
    m_trunk_trajectories->get("pos_z").addPoint(fix4, m_params.trunk_height);
    m_trunk_trajectories->get("pos_z").addPoint(fix5, m_params.trunk_height);

    /* Support trunk orientation */
    /* Construct a start_rotation as quaternion from Pose msg */
    start_rotation = tf::Quaternion(trunk_pose.orientation.x, trunk_pose.orientation.y,
                                    trunk_pose.orientation.z, trunk_pose.orientation.w);
    tf::Matrix3x3(start_rotation).getRPY(start_r, start_p, start_y);

    /* Also construct one for the target */
    target_rotation = tf::Quaternion(trunk_pose.orientation.x, trunk_pose.orientation.y,
                                     trunk_pose.orientation.z, trunk_pose.orientation.w);
    tf::Matrix3x3(target_rotation).getRPY(target_r, target_p, target_y);

    /* Add these quaternions in the same fashion as before to our splines (current, target, current) */
    m_trunk_trajectories->get("roll").addPoint(fix0, start_r);
    m_trunk_trajectories->get("roll").addPoint(fix1, start_r);
    m_trunk_trajectories->get("roll").addPoint(fix2, start_r - kick_foot_sign * m_params.trunk_kick_roll);
    m_trunk_trajectories->get("roll").addPoint(fix4, start_r - kick_foot_sign * m_params.trunk_kick_roll);
    m_trunk_trajectories->get("roll").addPoint(fix5, start_r);
    m_trunk_trajectories->get("pitch").addPoint(fix0, start_p);
    m_trunk_trajectories->get("pitch").addPoint(fix1, start_p);
    m_trunk_trajectories->get("pitch").addPoint(fix2, start_p);
    m_trunk_trajectories->get("pitch").addPoint(fix3, start_p - m_params.trunk_kick_pitch);
    m_trunk_trajectories->get("pitch").addPoint(fix4, start_p);
    m_trunk_trajectories->get("pitch").addPoint(fix5, start_p);
    m_trunk_trajectories->get("yaw").addPoint(fix0, start_y);
    m_trunk_trajectories->get("yaw").addPoint(fix5, start_y);
}

void KickEngine::init_trajectories() {
    m_trunk_trajectories = Trajectories();

    m_trunk_trajectories->add("pos_x");
    m_trunk_trajectories->add("pos_y");
    m_trunk_trajectories->add("pos_z");

    m_trunk_trajectories->add("roll");
    m_trunk_trajectories->add("pitch");
    m_trunk_trajectories->add("yaw");

    m_flying_trajectories = Trajectories();

    m_flying_trajectories->add("pos_x");
    m_flying_trajectories->add("pos_y");
    m_flying_trajectories->add("pos_z");

    m_flying_trajectories->add("roll");
    m_flying_trajectories->add("pitch");
    m_flying_trajectories->add("yaw");
}

bool KickEngine::is_left_kick() {
    return m_is_left_kick;
}

int KickEngine::get_percent_done() const {
    double duration = m_params.move_trunk_time + m_params.raise_foot_time + m_params.kick_time +
            m_params.move_back_time + m_params.lower_foot_time;
    return int(m_time / duration * 100);
}

void KickEngine::set_params(KickParams params) {
    m_params = params;
}
