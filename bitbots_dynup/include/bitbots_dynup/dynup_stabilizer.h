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


namespace bitbots_dynup {

class Stabilizer : public bitbots_splines::AbstractStabilizer<DynupResponse> {
 public:
  void init(moveit::core::RobotModelPtr kinematic_model);
  DynupResponse stabilize(const DynupResponse &response, const ros::Duration &dt) override;
  void useStabilizing(bool use);
  void useMinimalDisplacement(bool use);
  void setStabilizingWeight(double weight);
  void setRobotModel(moveit::core::RobotModelPtr model); 
  void reset() override;
 private:
  robot_state::RobotStatePtr goal_state_;

  robot_model::RobotModelPtr kinematic_model_;
  moveit::core::JointModelGroup *legs_joints_group_;

  bool use_stabilizing_;
  bool use_minimal_displacement_;
  double stabilizing_weight_;
};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
