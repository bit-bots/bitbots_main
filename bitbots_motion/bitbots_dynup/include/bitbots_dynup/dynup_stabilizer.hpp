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
  explicit Stabilizer(std::string ns);
  DynupResponse stabilize(const DynupResponse &response, const rclcpp::Duration &dt) override;
  void setRSoleToTrunk(geometry_msgs::msg::TransformStamped r_sole_to_trunk);
  void setParams(bitbots_dynup::Params::Engine::Stabilizer params);
  void reset() override;
  void setImu(sensor_msgs::msg::Imu::SharedPtr imu);
  bool isStable();

 private:
  sensor_msgs::msg::Imu::SharedPtr imu_;
  std::shared_ptr<control_toolbox::PidROS> pid_trunk_pitch_;
  std::shared_ptr<control_toolbox::PidROS> pid_trunk_roll_;
  std::shared_ptr<rclcpp::Node> pitch_node_;
  std::shared_ptr<rclcpp::Node> roll_node_;
  // Transform from r_sole frame to base_link frame, as we want to stabilize the base link.
  geometry_msgs::msg::TransformStamped r_sole_to_trunk_;

  bool stabilize_now_;

  bool is_stable_;
  double stable_threshold_;

  bool use_stabilizing_;
};

}  // namespace bitbots_dynup

#endif  // BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_STABILIZER_H_
