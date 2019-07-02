#ifndef BITBOTS_DYNAMIC_KICK_STABILIZER_H
#define BITBOTS_DYNAMIC_KICK_STABILIZER_H

#include <optional>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <bio_ik/bio_ik.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include "Visualizer.h"

typedef std::pair<std::vector<std::string>, std::vector<double>> JointGoals;

/**
 * The stabilizer is basically a wrapper around bio_ik and moveit
 */
class Stabilizer {
public:
    Stabilizer();

    Visualizer m_visualizer;
    geometry_msgs::Point m_cop_left;
    geometry_msgs::Point m_cop_right;

    /**
     * Calculate required motor positions to reach foot_goal with a foot while keeping the robot as stable as possible.
     * The stabilization itself is achieved by using moveit with bio_ik
     * @param is_left_kick Is the given foot_goal a goal which the left foot should reach
     * @param foot_goal Position which should be reached by the foot
     * @return JointGoals which describe required motor positions
     */
    std::optional<JointGoals> stabilize(bool is_left_kick, geometry_msgs::Point support_point, geometry_msgs::PoseStamped flying_foot_goal_pose, bool cop_support_point);
    void reset();
    void use_stabilizing(bool use);
    void use_minimal_displacement(bool use);
    void use_cop(bool use);
    void set_trunk_height(double height);
    void set_stabilizing_weight(double weight);
    void set_flying_weight(double weight);
    void set_trunk_orientation_weight(double weight);
    void set_trunk_height_weight(double weight);
    void set_p_factor(double factor);
    void set_i_factor(double factor);
private:
    robot_state::RobotStatePtr m_goal_state;
    planning_scene::PlanningScenePtr m_planning_scene;

    robot_model::RobotModelPtr m_kinematic_model;
    moveit::core::JointModelGroup* m_all_joints_group;
    moveit::core::JointModelGroup* m_legs_joints_group;

    double m_cop_error_sum_x;
    double m_cop_error_sum_y;

    bool m_use_stabilizing;
    bool m_use_minimal_displacement;
    bool m_use_cop;
    double m_trunk_height;
    double m_stabilizing_weight;
    double m_flying_weight;
    double m_trunk_orientation_weight;
    double m_trunk_height_weight;
    double m_p_factor;
    double m_i_factor;
};

#endif  // BITBOTS_DYNAMIC_KICK_STABILIZER_H
