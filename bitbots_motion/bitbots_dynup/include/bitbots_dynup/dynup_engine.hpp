#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_

#include <bitbots_dynup/msg/dynup_engine_debug.hpp>
#include <bitbots_splines/abstract_engine.hpp>
#include <bitbots_splines/pose_spline.hpp>
#include <bitbots_splines/smooth_spline.hpp>
#include <bitbots_splines/spline_container.hpp>
#include <cmath>
#include <dynup_parameters.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2/convert.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "dynup_stabilizer.hpp"

namespace bitbots_dynup {

class DynupEngine : public bitbots_splines::AbstractEngine<DynupRequest, DynupResponse> {
 public:
  explicit DynupEngine(rclcpp::Node::SharedPtr node, bitbots_dynup::Params::Engine params);

  void init(double arm_offset_y, double arm_offset_z);

  DynupResponse update(double dt) override;

  void setGoals(const DynupRequest &goals) override;

  /*
   * Publishes debug markers
   */
  void publishDebug();

  int getPercentDone() const override;

  double getDuration() const;

  DynupDirection getDirection();

  bool isStabilizingNeeded();

  bool isHeadZero();

  bitbots_splines::PoseSpline getRFootSplines() const;

  bitbots_splines::PoseSpline getLHandSplines() const;

  bitbots_splines::PoseSpline getRHandSplines() const;

  bitbots_splines::PoseSpline getLFootSplines() const;

  void setParams(bitbots_dynup::Params::Engine params);

  void reset() override;
  void reset(double time);

  void publishArrowMarker(std::string name_space, std::string frame, geometry_msgs::msg::Pose pose, float r, float g,
                          float b, float a);

 private:
  rclcpp::Node::SharedPtr node_;

  bitbots_dynup::Params::Engine params_;

  int marker_id_ = 1;
  double time_ = 0;
  double duration_ = 0;
  double arm_offset_y_ = 0;
  double arm_offset_z_ = 0;
  tf2::Transform offset_left_;
  tf2::Transform offset_right_;

  DynupDirection direction_ = DynupDirection::WALKREADY;

  bitbots_splines::PoseSpline l_foot_spline_;
  bitbots_splines::PoseSpline l_hand_spline_;
  bitbots_splines::PoseSpline r_foot_spline_;
  bitbots_splines::PoseSpline r_hand_spline_;

  DynupResponse goals_;
  std::shared_ptr<rclcpp::Node> walking_param_node_;
  std::shared_ptr<rclcpp::SyncParametersClient> walking_param_client_;

  rclcpp::Publisher<bitbots_dynup::msg::DynupEngineDebug>::SharedPtr pub_engine_debug_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_debug_marker_;

  /*
   * Helper method to extract the current pose of the left foot or the torso from the spline
   * @param spline The spline to get the pose from
   * @param foot true to get the left foot position, false to get the torso position
   * @returns the requested pose relative to the right foot
   */
  geometry_msgs::msg::PoseStamped getCurrentPose(bitbots_splines::PoseSpline spline, std::string frame_id);

  /*
   * Creates starting positions for the splines.
   */
  bitbots_splines::PoseSpline initializeSpline(geometry_msgs::msg::Pose pose, bitbots_splines::PoseSpline spline);

  /* Calculate the splines to get from lying on the front to squatting:
   * - move arms to front and pull legs
   * - get torso into 45°, pull foot under legs
   * - get into crouch position
   *
   * @return the time of the last splinepoint of this function, needed to concat rise or descend
   */
  double calcFrontSplines();

  /*
   * Calculate the splines to get from lying on the back to squatting
   *
   * @return the time of the last splinepoint of this function, needed to concat rise or descend
   */
  double calcBackSplines();

  /*
   * Calculate the splines to get up from a squatting position:
   *  - slowly stand up with stabilization
   *  - move arms in finish position
   *
   *  @return the time of the last splinepoint of this function, needed to concat rise or descend
   */
  double calcWalkreadySplines(double time = 0, double travel_time = 0);

  /*
   * Calculate the splines to get down to a squatting position:
   *  - slowly sit down with stabilization
   *
   *  @return the time of the last splinepoint of this function, needed to concat rise or descend
   */
  double calcDescendSplines(double time = 0);
};

}  // namespace bitbots_dynup

#endif  // BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_
