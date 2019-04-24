#include "bitbots_dynamic_kick/KickEngine.h"

KickEngine::KickEngine() {}

void KickEngine::reset() {
    m_time = 0;
    m_trajectories.reset();
}

void KickEngine::set_goal(const geometry_msgs::Pose& target_pose, double speed,
        const geometry_msgs::Pose& l_foot_pose, const geometry_msgs::Pose& r_foot_pose) {
    m_goal_pose = target_pose;
    m_speed = speed;
    m_time = 0;

    init_trajectories();
    calc_splines(l_foot_pose, r_foot_pose);
}

std::optional<JointGoals> KickEngine::tick(double dt) {
    if (m_trajectories) {
        /* Get should-be pose from planned splines (every axis) at current time */
        tf::Transform foot_pose;
        foot_pose.setOrigin({m_trajectories->get("foot_pos_x").pos(m_time),
                             m_trajectories->get("foot_pos_y").pos(m_time),
                             m_trajectories->get("foot_pos_z").pos(m_time)});
        tf::Quaternion q;
        /* Apparently, the axis order is different than expected */
        q.setEuler(m_trajectories->get("foot_pitch").pos(m_time),
                   m_trajectories->get("foot_roll").pos(m_time),
                   m_trajectories->get("foot_yaw").pos(m_time));
        foot_pose.setRotation(q);

        m_time += dt;

        /* Stabilize and return result */
        return m_stabilizer.stabilize(/* is_left_kick */ false, foot_pose);
    } else {
        return std::nullopt;
    }
}

void KickEngine::calc_splines(const geometry_msgs::Pose &target_pose, const geometry_msgs::Pose &r_foot_pose) {
    /* Add neccessary points to all splines so that they become smooth */
    m_trajectories->get("foot_pos_x").addPoint(0, r_foot_pose.position.x);
    m_trajectories->get("foot_pos_x").addPoint(1, m_goal_pose.position.x);
    m_trajectories->get("foot_pos_x").addPoint(2, r_foot_pose.position.x);

    m_trajectories->get("foot_pos_y").addPoint(0, r_foot_pose.position.y);
    m_trajectories->get("foot_pos_y").addPoint(1, m_goal_pose.position.y);
    m_trajectories->get("foot_pos_y").addPoint(2, r_foot_pose.position.y);

    m_trajectories->get("foot_pos_z").addPoint(0, r_foot_pose.position.z);
    m_trajectories->get("foot_pos_z").addPoint(1, m_goal_pose.position.z);
    m_trajectories->get("foot_pos_z").addPoint(2, r_foot_pose.position.z);

    tf::Quaternion start_rotation(r_foot_pose.orientation.x, r_foot_pose.orientation.y,
                                  r_foot_pose.orientation.z, r_foot_pose.orientation.w);
    double start_r, start_p, start_y;
    tf::Matrix3x3(start_rotation).getRPY(start_r, start_p, start_y);

    tf::Quaternion target_rotation(r_foot_pose.orientation.x, r_foot_pose.orientation.y,
                                   r_foot_pose.orientation.z, r_foot_pose.orientation.w);
    double target_r, target_p, target_y;
    tf::Matrix3x3(target_rotation).getRPY(target_r, target_p, target_y);

    m_trajectories->get("foot_roll").addPoint(0, start_r);
    m_trajectories->get("foot_roll").addPoint(0, target_r);
    m_trajectories->get("foot_roll").addPoint(2, start_r);

    m_trajectories->get("foot_pitch").addPoint(0, start_p);
    m_trajectories->get("foot_pitch").addPoint(0, target_p);
    m_trajectories->get("foot_pitch").addPoint(2, start_p);

    m_trajectories->get("foot_yaw").addPoint(0, start_y);
    m_trajectories->get("foot_yaw").addPoint(0, target_y);
    m_trajectories->get("foot_yaw").addPoint(2, start_y);
}

void KickEngine::init_trajectories() {
    m_trajectories = Trajectories();

    m_trajectories->add("foot_pos_x");
    m_trajectories->add("foot_pos_y");
    m_trajectories->add("foot_pos_z");

    m_trajectories->add("foot_roll");
    m_trajectories->add("foot_pitch");
    m_trajectories->add("foot_yaw");
}

bool KickEngine::is_left_kick() {
    return false;
}
