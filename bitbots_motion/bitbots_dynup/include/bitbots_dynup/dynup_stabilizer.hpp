#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_

#include <rot_conv/rot_conv.h>

#include <Eigen/Geometry>
#include <bitbots_splines/abstract_stabilizer.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <optional>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "dynup_parameters.hpp"
#include "dynup_utils.hpp"
namespace bitbots_dynup {

class Stabilizer : public bitbots_splines::AbstractStabilizer<DynupResponse> {
 public:
  explicit Stabilizer(rclcpp::Node::SharedPtr node, bitbots_dynup::Params::Stabilizer params);
  DynupResponse stabilize(const DynupResponse &response, const rclcpp::Duration &dt) override;
  void setRSoleToTrunk(geometry_msgs::msg::TransformStamped r_sole_to_trunk);
  void setParams(bitbots_dynup::Params::Stabilizer params);
  void reset() override;
  void setImu(sensor_msgs::msg::Imu::SharedPtr imu);
  bool isStable();

 private:
  bitbots_dynup::Params::Stabilizer params_;
  control_toolbox::PidROS pid_trunk_pitch_;
  control_toolbox::PidROS pid_trunk_roll_;
  // Transform from r_sole frame to base_link frame, as we want to stabilize the base link.
  std::optional<geometry_msgs::msg::TransformStamped> r_sole_to_trunk_;
  sensor_msgs::msg::Imu::SharedPtr imu_;

  bool is_stable_ = true;
};

}  // namespace bitbots_dynup

#endif  // BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
