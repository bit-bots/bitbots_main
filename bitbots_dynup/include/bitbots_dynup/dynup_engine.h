#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_

#include <string>
#include <optional>
#include <cmath>
#include <bitbots_splines/smooth_spline.h>
#include <bitbots_splines/spline_container.h>
#include <bitbots_splines/pose_spline.h>
#include <bitbots_splines/abstract_engine.h>
#include <bitbots_dynup/DynUpConfig.h>
#include <bitbots_dynup/DynupEngineDebug.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "dynup_stabilizer.h"

namespace bitbots_dynup {



class DynupEngine : public bitbots_splines::AbstractEngine<DynupRequest, DynupResponse> {
 public:
  DynupEngine();

  void init(double arm_offset_y, double arm_offset_z);

  DynupResponse update(double dt) override;

  /*
   * Add current position, target position and current position to splines so that they describe a smooth
   * curve to the ball and back
   */
  void setGoals(const DynupRequest &goals) override;

  /*
   * Publishes debug markers
   */
  void publishDebug();

  int getPercentDone() const override;

  bool isStabilizingNeeded() const;

  bool isHeadZero() const;

  bitbots_splines::PoseSpline getRFootSplines() const;

  bitbots_splines::PoseSpline getLHandSplines() const;

  bitbots_splines::PoseSpline getRHandSplines() const;

  bitbots_splines::PoseSpline getLFootSplines() const;

  void setParams(DynUpConfig params);

  void reset() override;

  void publishArrowMarker(std::string name_space,
                          std::string frame,
                          geometry_msgs::Pose pose,
                          float r,
                          float g,
                          float b,
                          float a);

 private:
  int marker_id_;
  double time_;
  double duration_;
  double shoulder_offset_y_;
  double arm_offset_y_;
  double arm_offset_z_;
  tf2::Transform offset_left_;
  tf2::Transform offset_right_;
  int direction_;

  bitbots_splines::PoseSpline l_foot_spline_;
  bitbots_splines::PoseSpline l_hand_spline_;
  bitbots_splines::PoseSpline r_foot_spline_;
  bitbots_splines::PoseSpline r_hand_spline_;
  DynUpConfig params_;

  DynupResponse goals_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;

  ros::Publisher pub_engine_debug_;
  ros::Publisher pub_debug_marker_;

  /*
   * Helper method to extract the current pose of the left foot or the torso from the spline
   * @param spline The spline to get the pose from
   * @param foot true to get the left foot position, false to get the torso position
   * @returns the requested pose relative to the right foot
   */
  geometry_msgs::PoseStamped getCurrentPose(bitbots_splines::PoseSpline spline, std::string frame_id);

  /*
   * Creates starting positions for the splines.
   */
  bitbots_splines::PoseSpline initializeSpline(geometry_msgs::Pose pose, bitbots_splines::PoseSpline spline);

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
  double calcRiseSplines(double time);

  /*
   * Calculate the splines to get down to a squatting position:
   *  - slowly sit down with stabilization
   *
   *  @return the time of the last splinepoint of this function, needed to concat rise or descend
   */
  double calcDescendSplines(double time);


};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_
