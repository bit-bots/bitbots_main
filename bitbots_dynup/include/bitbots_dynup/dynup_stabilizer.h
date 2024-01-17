#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_

#include <optional>
#include <sensor_msgs/msg/imu.hpp>
#include <bitbots_splines/abstract_stabilizer.h>
#include "dynup_utils.h"
#include <control_toolbox/pid_ros.hpp>
#include <rot_conv/rot_conv.h>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace bitbots_dynup {

class Stabilizer : public bitbots_splines::AbstractStabilizer<DynupResponse> {
 public:
  explicit Stabilizer(std::string ns);
  DynupResponse stabilize(const DynupResponse &response, const rclcpp::Duration &dt) override;
  void setRSoleToTrunk(geometry_msgs::msg::TransformStamped r_sole_to_trunk);
  void setParams(std::map<std::string, rclcpp::Parameter> params);
  void reset() override;
  void setImu(sensor_msgs::msg::Imu::SharedPtr imu);
  bool isStable();

 private:
  sensor_msgs::msg::Imu::SharedPtr imu_;
  std::shared_ptr<control_toolbox::PidROS> pid_trunk_pitch_;
  std::shared_ptr<control_toolbox::PidROS> pid_trunk_roll_;
  std::shared_ptr<rclcpp::Node> pitch_node_;
  std::shared_ptr<rclcpp::Node> roll_node_;
  //Transform from r_sole frame to base_link frame, as we want to stabilize the base link.
  geometry_msgs::msg::TransformStamped r_sole_to_trunk_;

  bool stabilize_now_;

  bool is_stable_;
  double stable_threshold_;

  bool use_stabilizing_;
};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
