#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_

#include <string>
#include <optional>
#include <bitbots_splines/SmoothSpline.hpp>
#include <bitbots_splines/SplineContainer.hpp>
#include <bitbots_splines/pose_spline.h>
#include <bitbots_splines/abstract_engine.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "dynup_stabilizer.h"

namespace bitbots_dynup {

struct DynUpParams {
  double leg_min_length;
  double arm_max_length;
  double time_foot_close;
  double time_hands_front;
  double time_hands_side;
  double time_foot_ground;
  double time_torso_45;

  double foot_distance;
  double rise_time;
  double trunk_x;
  double trunk_height;
  double trunk_pitch;
};

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

  void setParams(DynUpParams params);

  void reset() override;
 private:
  double time_;

  bitbots_splines::PoseSpline foot_spline_;
  bitbots_splines::PoseSpline hand_spline_;
  bitbots_splines::PoseSpline trunk_spline_;
  DynUpParams params_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;

  /*
   * Helper method to extract the current pose of the left foot or the torso from the spline
   * @param spline The spline to get the pose from
   * @param foot true to get the left foot position, false to get the torso position
   * @returns the requested pose relative to the right foot
   */
  geometry_msgs::PoseStamped getCurrentPose(bitbots_splines::PoseSpline spline, bool foot);

  /* Calculate the splines to get from lying on the front to squatting:
   * - move arms to frint and pull legs
   * - get torso into 45°, pull foot under legs
   * - get into crouch position
   */
  void calcFrontSplines(geometry_msgs::Pose l_foot_pose, geometry_msgs::Pose l_hand_pose);

  /*
   * Calculate the splines to get from lying on the back to squatting
   */
  void calcBackSplines();

  /*
   * Calculate the splines to get up from a squatting position:
  *  - slowly stand up with stabilization
  *  - move arms in finish position
  */
  void calcSquatSplines(geometry_msgs::Pose l_foot_pose, geometry_msgs::Pose trunk_pose);

};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_
