#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_STABILIZER_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_STABILIZER_H_

#include <optional>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <bio_ik/bio_ik.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>

namespace bitbots_dynup {

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
  std::optional<JointGoals> stabilize(geometry_msgs::Point support_point,
                                      geometry_msgs::PoseStamped &l_foot_goal_pose,
                                      geometry_msgs::PoseStamped &trunk_goal_pose);
  void useStabilizing(bool use);
  void useMinimalDisplacement(bool use);
  void setStabilizingWeight(double weight);
  void reset();
 private:
  robot_state::RobotStatePtr goal_state_;

  robot_model::RobotModelPtr kinematic_model_;
  moveit::core::JointModelGroup *legs_joints_group_;

  bool use_stabilizing_;
  bool use_minimal_displacement_;
  double stabilizing_weight_;
};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_STABILIZER_H_
