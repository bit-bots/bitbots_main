#include "bitbots_dynamic_kick/KickEngine.h"

KickEngine::KickEngine() {}

void KickEngine::reset() {
    m_time = 0;
    // TODO Reset all other goal variables (depends on whether we use std::option)
}

void KickEngine::set_goal(const geometry_msgs::Pose& target_pose, double speed,
        const geometry_msgs::Pose& l_foot_pose, const geometry_msgs::Pose& r_foot_pose) {
    m_goal_pose = target_pose;
    m_speed = speed;
    m_time = 0;

    init_trajectories();
    calc_splines(l_foot_pose, r_foot_pose);
}

bool KickEngine::tick(double dt, JointGoals& goals) {
    // Get should-be pose from planned splines (every axis) at current time
    geometry_msgs::Pose foot_pose;
    foot_pose.position.x = m_trajectories.get("foot_pos_x").pos(m_time);
    foot_pose.position.y = m_trajectories.get("foot_pos_y").pos(m_time);
    foot_pose.position.z = m_trajectories.get("foot_pos_z").pos(m_time);
    foot_pose.orientation.w = 1;

    // Stabilize and return result
    bool success = m_stabilizer.stabilize(/* is_left_kick */ false, foot_pose, goals);
    m_time += dt;
    return success;
}

void KickEngine::calc_splines(const geometry_msgs::Pose &target_pose, const geometry_msgs::Pose &r_foot_pose) {
    m_trajectories.get("foot_pos_x").addPoint(0, r_foot_pose.position.x);
    m_trajectories.get("foot_pos_x").addPoint(1, m_goal_pose.position.x);

    m_trajectories.get("foot_pos_y").addPoint(0, r_foot_pose.position.y);
    m_trajectories.get("foot_pos_y").addPoint(1, m_goal_pose.position.y);

    m_trajectories.get("foot_pos_z").addPoint(0, r_foot_pose.position.z);
    m_trajectories.get("foot_pos_z").addPoint(1, m_goal_pose.position.z);
}

void KickEngine::init_trajectories() {
    m_trajectories = Trajectories();

    m_trajectories.add("foot_pos_x");
    m_trajectories.add("foot_pos_y");
    m_trajectories.add("foot_pos_z");

    m_trajectories.add("foot_rot_x");
    m_trajectories.add("foot_rot_y");
    m_trajectories.add("foot_rot_z");
}

bool KickEngine::is_left_kick() {
    return false;
}
