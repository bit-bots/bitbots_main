#ifndef BITBOTS_DYNAMIC_KICK_STABILIZER_H
#define BITBOTS_DYNAMIC_KICK_STABILIZER_H

#include <optional>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <bio_ik/bio_ik.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>

typedef std::pair<std::vector<std::string>, std::vector<double>> JointGoals;

/**
 * The stabilizer is basically a wrapper around bio_ik and moveit
 */
class Stabilizer {
public:
    Stabilizer();

    /**
     * Calculate required motor positions to reach foot_goal with a foot while keeping the robot as stable as possible.
     * The stabilization itself is achieved by using moveit with bio_ik
     * @param is_left_kick Is the given foot_goal a goal which the left foot should reach
     * @param foot_goal Position which should be reached by the foot
     * @return JointGoals which describe required motor positions
     */
    std::optional<JointGoals> stabilize(bool is_left_kick, geometry_msgs::PoseStamped trunk_goal_lsole, geometry_msgs::PoseStamped flying_foot_goal);

private:
    robot_state::RobotStatePtr m_goal_state;
    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_listener;

    moveit::core::JointModelGroup* m_all_joints_group;
    moveit::core::JointModelGroup* m_legs_joints_group;
    moveit::core::JointModelGroup* m_lleg_joints_group;
    moveit::core::JointModelGroup* m_rleg_joints_group;


};

#endif  // BITBOTS_DYNAMIC_KICK_STABILIZER_H
