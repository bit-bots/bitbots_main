#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_

#include <optional>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <bitbots_splines/abstract_stabilizer.h>
#include "kick_utils.h"
#include "visualizer.h"
#include <control_toolbox/pid_ros.hpp>

namespace bitbots_dynamic_kick {

class Stabilizer :
    public bitbots_splines::AbstractStabilizer<KickPositions> {
 public:
  Stabilizer(std::string ns);

  geometry_msgs::msg::Point cop_left;
  geometry_msgs::msg::Point cop_right;

  /**
   * Calculate required IK goals to reach foot_goal with a foot while keeping the robot as stable as possible.
   * @param positions a description of the required positions
   * @return BioIK Options that can be used by an instance of AbstractIK
   */

  KickPositions stabilize(const KickPositions &positions, const rclcpp::Duration &dt) override;
  void reset() override;
  void useCop(bool use);
  void setRobotModel(moveit::core::RobotModelPtr model);
 private:
  moveit::core::RobotModelPtr kinematic_model_;
    std::shared_ptr<control_toolbox::PidROS> pid_trunk_fused_pitch_;
    std::shared_ptr<control_toolbox::PidROS> pid_trunk_fused_roll_;
    std::shared_ptr<rclcpp::Node> pitch_node_;
    std::shared_ptr<rclcpp::Node> roll_node_;

  bool use_cop_;
};
}

#endif  //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_
