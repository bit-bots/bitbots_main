#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_

#include <optional>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <bitbots_splines/abstract_stabilizer.h>
#include "dynup_utils.h"
#include <moveit/robot_state/robot_state.h>
#include <tf2_ros/transform_listener.h>
#include <control_toolbox/pid.h>

namespace bitbots_dynup {

class Stabilizer : public bitbots_splines::AbstractStabilizer<DynupResponse> {
 public:
  void init(moveit::core::RobotModelPtr kinematic_model);
  DynupResponse stabilize(const DynupResponse &response, const ros::Duration &dt) override;
  void setStabilizeNow(bool now);
  void setTransforms(geometry_msgs::TransformStamped to_trunk, geometry_msgs::TransformStamped from_trunk);
  void useStabilizing(bool use);
  void useMinimalDisplacement(bool use);
  void setStabilizingWeight(double weight);
  void setRobotModel(moveit::core::RobotModelPtr model); 
  void reset() override;
  geometry_msgs::Point cop_;
  control_toolbox::Pid pid_trunk_pitch_;
  control_toolbox::Pid pid_trunk_roll_;

 private:
  robot_state::RobotStatePtr goal_state_;
  robot_model::RobotModelPtr kinematic_model_;
  geometry_msgs::TransformStamped to_trunk_, from_trunk_;

  bool stabilize_now_;

  bool use_stabilizing_;
  bool use_minimal_displacement_;
  double stabilizing_weight_;
};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
