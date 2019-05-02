#include "bitbots_dynamic_kick/KickEngine.h"

KickEngine::KickEngine() {}

void KickEngine::reset() {
    m_time = 0;
    m_support_trajectories.reset();
    m_flying_trajectories.reset();
}

void KickEngine::set_goal(const geometry_msgs::Pose& target_pose, double speed,
                          const geometry_msgs::Pose& l_foot_pose, const geometry_msgs::Pose& r_foot_pose) {
    /* Save given goals because we reuse them later */
    m_goal_pose = target_pose;
    m_speed = speed;
    m_time = 0;

    /* Plan new splines according to new goal */
    init_trajectories();
    calc_splines(target_pose, r_foot_pose, l_foot_pose);
}

std::optional<JointGoals> KickEngine::tick(double dt) {
    /* Only do an actual tick when splines are present */
    if (m_support_trajectories && m_flying_trajectories) {
        /* Get should-be pose from planned splines (every axis) at current time */
        tf::Transform support_foot_pose = get_current_pose(m_support_trajectories.value());
        tf::Transform flying_foot_pose = get_current_pose(m_flying_trajectories.value());

        std::cout << m_time << std::endl;
        m_time += dt;

        /* Stabilize and return result */
        return m_stabilizer.stabilize(/* is_left_kick */ false, support_foot_pose, flying_foot_pose);
    } else {
        return std::nullopt;
    }
}

tf::Transform KickEngine::get_current_pose(Trajectories spline_container) {
    tf::Transform foot_pose;
    foot_pose.setOrigin({spline_container.get("foot_pos_x").pos(m_time),
                         spline_container.get("foot_pos_y").pos(m_time),
                         spline_container.get("foot_pos_z").pos(m_time)});
    tf::Quaternion q;
    /* Apparently, the axis order is different than expected */
    q.setEuler(spline_container.get("foot_pitch").pos(m_time),
               spline_container.get("foot_roll").pos(m_time),
               spline_container.get("foot_yaw").pos(m_time));
    foot_pose.setRotation(q);
    return foot_pose;
}

void KickEngine::calc_splines(const geometry_msgs::Pose &target_pose, const geometry_msgs::Pose &r_foot_pose,
                              const geometry_msgs::Pose &l_foot_pose) {
    /*
     * Add current position, target position and current position to splines so that they describe a smooth
     * curve to the and back
     */
    double trunk_movement = 0.06;  // meters to move trunk to the left (i.e. support foot to the right)
    double foot_rise = 0.08;        // how much the foot is raised before kicking
    double kick_distance = 0.12;    // kick distance to the front
    /* Splines:
     * - stand
     * - move trunk
     * - raise foot
     * - kick
     * - move foot back
     * - lower foot and move trunk
     */

    /* Flying foot position */
    m_flying_trajectories->get("foot_pos_x").addPoint(0, r_foot_pose.position.x);
    m_flying_trajectories->get("foot_pos_x").addPoint(1, r_foot_pose.position.x);
    m_flying_trajectories->get("foot_pos_x").addPoint(2, r_foot_pose.position.x);
    m_flying_trajectories->get("foot_pos_x").addPoint(3, r_foot_pose.position.x + kick_distance);
    m_flying_trajectories->get("foot_pos_x").addPoint(4, r_foot_pose.position.x);
    m_flying_trajectories->get("foot_pos_x").addPoint(5, r_foot_pose.position.x);

    m_flying_trajectories->get("foot_pos_y").addPoint(0, r_foot_pose.position.y);
    m_flying_trajectories->get("foot_pos_y").addPoint(1, r_foot_pose.position.y - trunk_movement);
    m_flying_trajectories->get("foot_pos_y").addPoint(2, r_foot_pose.position.y - trunk_movement);
    m_flying_trajectories->get("foot_pos_y").addPoint(3, r_foot_pose.position.y - trunk_movement);
    m_flying_trajectories->get("foot_pos_y").addPoint(4, r_foot_pose.position.y - trunk_movement);
    m_flying_trajectories->get("foot_pos_y").addPoint(5, r_foot_pose.position.y);

    m_flying_trajectories->get("foot_pos_z").addPoint(0, r_foot_pose.position.z);
    m_flying_trajectories->get("foot_pos_z").addPoint(1, r_foot_pose.position.z);
    m_flying_trajectories->get("foot_pos_z").addPoint(2, r_foot_pose.position.z + foot_rise);
    m_flying_trajectories->get("foot_pos_z").addPoint(3, r_foot_pose.position.z + foot_rise);
    m_flying_trajectories->get("foot_pos_z").addPoint(4, r_foot_pose.position.z + foot_rise);
    m_flying_trajectories->get("foot_pos_z").addPoint(5, r_foot_pose.position.z);

    /* Flying foot orientation */
    /* Construct a start_rotation as quaternion from Pose msg */
    tf::Quaternion start_rotation(r_foot_pose.orientation.x, r_foot_pose.orientation.y,
                                  r_foot_pose.orientation.z, r_foot_pose.orientation.w);
    double start_r, start_p, start_y;
    tf::Matrix3x3(start_rotation).getRPY(start_r, start_p, start_y);

    /* Also construct one for the target */
    tf::Quaternion target_rotation(r_foot_pose.orientation.x, r_foot_pose.orientation.y,
                                   r_foot_pose.orientation.z, r_foot_pose.orientation.w);
    double target_r, target_p, target_y;
    tf::Matrix3x3(target_rotation).getRPY(target_r, target_p, target_y);

    /* Add these quaternions in the same fashion as before to our splines (current, target, current) */
    m_flying_trajectories->get("foot_roll").addPoint(0, start_r);
    m_flying_trajectories->get("foot_pitch").addPoint(0, start_p);
    m_flying_trajectories->get("foot_yaw").addPoint(0, start_y);

    /* Support foot position */
    m_support_trajectories->get("foot_pos_x").addPoint(0, l_foot_pose.position.x);
    m_support_trajectories->get("foot_pos_x").addPoint(1, l_foot_pose.position.x);
    m_support_trajectories->get("foot_pos_x").addPoint(2, l_foot_pose.position.x);
    m_support_trajectories->get("foot_pos_x").addPoint(3, l_foot_pose.position.x);
    m_support_trajectories->get("foot_pos_x").addPoint(4, l_foot_pose.position.x);
    m_support_trajectories->get("foot_pos_x").addPoint(5, l_foot_pose.position.x);

    m_support_trajectories->get("foot_pos_y").addPoint(0, l_foot_pose.position.y);
    m_support_trajectories->get("foot_pos_y").addPoint(1, l_foot_pose.position.y - trunk_movement);
    m_support_trajectories->get("foot_pos_y").addPoint(2, l_foot_pose.position.y - trunk_movement);
    m_support_trajectories->get("foot_pos_y").addPoint(3, l_foot_pose.position.y - trunk_movement);
    m_support_trajectories->get("foot_pos_y").addPoint(4, l_foot_pose.position.y - trunk_movement);
    m_support_trajectories->get("foot_pos_y").addPoint(5, l_foot_pose.position.y);

    m_support_trajectories->get("foot_pos_z").addPoint(0, l_foot_pose.position.z);
    m_support_trajectories->get("foot_pos_z").addPoint(1, l_foot_pose.position.z);
    m_support_trajectories->get("foot_pos_z").addPoint(2, l_foot_pose.position.z);
    m_support_trajectories->get("foot_pos_z").addPoint(3, l_foot_pose.position.z);
    m_support_trajectories->get("foot_pos_z").addPoint(4, l_foot_pose.position.z);
    m_support_trajectories->get("foot_pos_z").addPoint(5, l_foot_pose.position.z);

    /* Support foot orientation */
    /* Construct a start_rotation as quaternion from Pose msg */
    start_rotation = tf::Quaternion(l_foot_pose.orientation.x, l_foot_pose.orientation.y,
                                    l_foot_pose.orientation.z, l_foot_pose.orientation.w);
    tf::Matrix3x3(start_rotation).getRPY(start_r, start_p, start_y);

    /* Also construct one for the target */
    target_rotation = tf::Quaternion(l_foot_pose.orientation.x, l_foot_pose.orientation.y,
                                     l_foot_pose.orientation.z, l_foot_pose.orientation.w);
    tf::Matrix3x3(target_rotation).getRPY(target_r, target_p, target_y);

    /* Add these quaternions in the same fashion as before to our splines (current, target, current) */
    m_support_trajectories->get("foot_roll").addPoint(0, start_r);
    m_support_trajectories->get("foot_pitch").addPoint(0, start_p);
    m_support_trajectories->get("foot_yaw").addPoint(0, start_y);
}

void KickEngine::init_trajectories() {
    m_support_trajectories = Trajectories();

    m_support_trajectories->add("foot_pos_x");
    m_support_trajectories->add("foot_pos_y");
    m_support_trajectories->add("foot_pos_z");

    m_support_trajectories->add("foot_roll");
    m_support_trajectories->add("foot_pitch");
    m_support_trajectories->add("foot_yaw");

    m_flying_trajectories = Trajectories();

    m_flying_trajectories->add("foot_pos_x");
    m_flying_trajectories->add("foot_pos_y");
    m_flying_trajectories->add("foot_pos_z");

    m_flying_trajectories->add("foot_roll");
    m_flying_trajectories->add("foot_pitch");
    m_flying_trajectories->add("foot_yaw");
}

bool KickEngine::is_left_kick() {
    return false;
}

int KickEngine::get_percent_done() const {
    return int(m_time / 6.0 * 100);
}
