#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_

#include <optional>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <bitbots_splines/abstract_stabilizer.h>
#include "dynup_utils.h"
#include <moveit/robot_state/robot_state.h>
#include <tf2_ros/transform_listener.h>


namespace bitbots_dynup {

/**
 * The stabilizer is basically a wrapper around bio_ik and moveit
 */
class Stabilizer : public bitbots_splines::AbstractStabilizer<DynupResponse> {
 public:
  void init(moveit::core::RobotModelPtr kinematic_model);
  DynupResponse stabilize(const DynupResponse &response, const ros::Duration &dt) override;
  void setStabilizeNow(bool now);
  void setCoP(geometry_msgs::Point cop);
  void setPFactor(double factor_x, double factor_y);
  void setIFactor(double factor_x, double factor_y);
  void setDFactor(double factor_x, double factor_y);
  void setTransforms(geometry_msgs::TransformStamped to_trunk, geometry_msgs::TransformStamped from_trunk);
  void useStabilizing(bool use);
  void useMinimalDisplacement(bool use);
  void setStabilizingWeight(double weight);
  void setRobotModel(moveit::core::RobotModelPtr model); 
  void reset() override;
 private:


  double cop_x_error_sum_, cop_y_error_sum_, cop_x_error_, cop_y_error_, p_x_factor_, p_y_factor_, i_x_factor_,
         i_y_factor_, d_x_factor_, d_y_factor_;

  robot_state::RobotStatePtr goal_state_;

  robot_model::RobotModelPtr kinematic_model_;

  geometry_msgs::TransformStamped to_trunk_, from_trunk_;

  geometry_msgs::Point cop_;

  bool stabilize_now_;

  bool use_stabilizing_;
  bool use_minimal_displacement_;
  double stabilizing_weight_;
};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
