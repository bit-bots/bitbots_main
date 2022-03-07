#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_

#include <optional>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_listener.h>
#include <bitbots_splines/abstract_stabilizer.h>
#include "dynup_utils.h"
#include <moveit/robot_state/robot_state.h>
#include <tf2_ros/transform_listener.h>
#include <control_toolbox/pid.hpp>
#include <rot_conv/rot_conv.h>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace bitbots_dynup {

class Stabilizer : public bitbots_splines::AbstractStabilizer<DynupResponse> {
 public:
  void init(moveit::core::RobotModelPtr kinematic_model);
  DynupResponse stabilize(const DynupResponse &response, const rclcpp::Duration &dt) override;
  void setRSoleToTrunk(geometry_msgs::msg::TransformStamped r_sole_to_trunk);
  void setParams(std::map<std::string, rclcpp::Parameter> params);
  void setRobotModel(moveit::core::RobotModelPtr model);
  void reset() override;
  void setImu(sensor_msgs::msg::Imu imu);
  bool isStable();

 private:
  sensor_msgs::msg::Imu imu_;
  control_toolbox::Pid pid_trunk_pitch_;
  control_toolbox::Pid pid_trunk_roll_;
  moveit::core::RobotStatePtr goal_state_;
  moveit::core::RobotModelPtr kinematic_model_;
  //Transform from r_sole frame to base_link frame, as we want to stabilize the base link.
  geometry_msgs::msg::TransformStamped r_sole_to_trunk_;

  bool stabilize_now_;

  bool is_stable_;
  double stable_threshold_;

  bool use_stabilizing_;
};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
