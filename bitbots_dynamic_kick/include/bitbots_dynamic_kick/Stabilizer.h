#ifndef BITBOTS_DYNAMIC_KICK_STABILIZER_H
#define BITBOTS_DYNAMIC_KICK_STABILIZER_H

#include <optional>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <bio_ik/bio_ik.h>
#include <geometry_msgs/Pose.h>

typedef std::pair<std::vector<std::string>, std::vector<double>> JointGoals;

class Stabilizer {
public:
    Stabilizer();
    std::optional<JointGoals> stabilize(bool is_left_kick, geometry_msgs::Pose foot_goal);

private:
    robot_state::RobotStatePtr m_goal_state;

    moveit::core::JointModelGroup* m_all_joints_group;
    moveit::core::JointModelGroup* m_legs_joints_group;
    moveit::core::JointModelGroup* m_lleg_joints_group;
    moveit::core::JointModelGroup* m_rleg_joints_group;


};

#endif  // BITBOTS_DYNAMIC_KICK_STABILIZER_H
