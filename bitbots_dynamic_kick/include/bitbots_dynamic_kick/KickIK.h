#ifndef BITBOTS_DYNAMIC_KICK_KICKIK_H
#define BITBOTS_DYNAMIC_KICK_KICKIK_H

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <bitbots_splines/AbstractIK.h>

class KickIK : public AbstractIK {
public:
    KickIK() = default;
    void init(moveit::core::RobotModelPtr kinematic_model) override;
    JointGoals calculate(std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) override;
    void reset() override;

private:
    robot_state::RobotStatePtr m_goal_state;
    planning_scene::PlanningScenePtr m_planning_scene;
    moveit::core::JointModelGroup* m_legs_joints_group;
};

#endif //BITBOTS_DYNAMIC_KICK_KICKIK_H
