#ifndef BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_
#define BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_

#include <optional>
#include <bio_ik/bio_ik.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <bitbots_splines/abstract_stabilizer.h>
#include "kick_utils.h"
#include "visualizer.h"

namespace bitbots_dynamic_kick {

class Stabilizer :
    public bitbots_splines::AbstractStabilizer<KickPositions> {
 public:
  Stabilizer();

  geometry_msgs::Point cop_left;
  geometry_msgs::Point cop_right;

  /**
   * Calculate required IK goals to reach foot_goal with a foot while keeping the robot as stable as possible.
   * @param positions a description of the required positions
   * @return BioIK Options that can be used by an instance of AbstractIK
   */

  std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> stabilize(const KickPositions &positions)
  override;
  void reset()
  override;
  void useStabilizing(bool use);
  void useMinimalDisplacement(bool use);
  void useCop(bool use);
  void setTrunkHeight(double height);
  void setStabilizingWeight(double weight);
  void setFlyingWeight(double weight);
  void setTrunkOrientationWeight(double weight);
  void setTrunkHeightWeight(double weight);
  void setPFactor(double factor_x, double factor_y);
  void setIFactor(double factor_x, double factor_y);
  void setDFactor(double factor_x, double factor_y);
  tf2::Vector3 getStabilizingTarget() const;
  void setRobotModel(moveit::core::RobotModelPtr model);
 private:
  moveit::core::RobotModelPtr kinematic_model_;
  tf2::Vector3 stabilizing_target_;
  double cop_x_error_sum_;
  double cop_y_error_sum_;
  double cop_x_error_;
  double cop_y_error_;

  bool use_stabilizing_;
  bool use_minimal_displacement_;
  bool use_cop_;
  double trunk_height_;
  double stabilizing_weight_;
  double flying_weight_;
  double trunk_orientation_weight_;
  double trunk_height_weight_;
  double p_x_factor_;
  double p_y_factor_;
  double i_x_factor_;
  double i_y_factor_;
  double d_x_factor_;
  double d_y_factor_;
};
}

#endif  //BITBOTS_DYNAMIC_KICK_INCLUDE_BITBOTS_DYNAMIC_KICK_STABILIZER_H_
