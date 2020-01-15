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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "dynup_stabilizer.h"

namespace bitbots_dynup {



class DynupEngine : public bitbots_splines::AbstractEngine<DynupRequest, DynupResponse> {
 public:
  DynupEngine();

  DynupResponse update(double dt) override;

  /*
   * Add current position, target position and current position to splines so that they describe a smooth
   * curve to the ball and back
   */
  void setGoals(const DynupRequest &goals) override;

  int getPercentDone() const override;

  void setParams(DynUpConfig params);

  void reset() override;
 private:
  double time_;
  double duration_;

  bitbots_splines::PoseSpline foot_spline_;
  bitbots_splines::PoseSpline l_hand_spline_;
  bitbots_splines::PoseSpline trunk_spline_;
  bitbots_splines::PoseSpline r_hand_spline_;
  DynUpConfig params_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;

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
  void initializeSplines(geometry_msgs::Pose pose, bitbots_splines::PoseSpline spline);

  /* Calculate the splines to get from lying on the front to squatting:
   * - move arms to frint and pull legs
   * - get torso into 45°, pull foot under legs
   * - get into crouch position
   */
  void calcFrontSplines();

  /*
   * Calculate the splines to get from lying on the back to squatting
   */
  void calcBackSplines();

  /*
   * Calculate the splines to get up from a squatting position:
  *  - slowly stand up with stabilization
  *  - move arms in finish position
  */
  void calcSquatSplines(double time);
};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_
