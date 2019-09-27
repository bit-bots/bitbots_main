#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_

#include <string>
#include <optional>
#include <bitbots_splines/SmoothSpline.hpp>
#include <bitbots_splines/SplineContainer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "stabilizer.h"
#include <math.h>

namespace bitbots_dynup {

typedef bitbots_splines::SplineContainer<bitbots_splines::SmoothSpline> Trajectories;

class DynUpParams {
 public:
  double leg_min_length;
  double arm_max_length;
  double time_foot_close;
  double time_hands_front;
  double time_foot_ground;
  double time_torso_45;

  double foot_distance;
  double rise_time;
  double trunk_x;
  double trunk_height;
  double trunk_pitch;
};

/**
 * TODO
 */
class DynupEngine {
 public:
  DynupEngine();

  /**
   * Do one iteration of spline-progress-updating. This means that whenever tick() is called,
   *      new position goals are retrieved from previously calculated splines, stabilized and transformed into
   *      JointGoals
   * @param dt Passed delta-time between last call to tick() and now. Measured in seconds
   * @return New motor goals only if a goal is currently set, position extractions from splines was possible and
   *      bio_ik was able to compute valid motor positions
   */
  std::optional<JointGoals> tick(double dt);

  void start(bool front, geometry_msgs::Pose l_foot_pose, geometry_msgs::Pose trunk_pose);

  int getPercentDone() const;

  void setParams(DynUpParams params);

  void reset();

  Stabilizer stabilizer;
 private:
  double time_;

  std::optional<Trajectories> foot_trajectories_, hand_trajectories_, trunk_trajectories_;
  DynUpParams params_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;

  geometry_msgs::PoseStamped getCurrentPose(Trajectories spline_container, bool foot);

  void calcFrontSplines();

  void calcBackSplines();

  void calcSquatSplines(geometry_msgs::Pose l_foot_pose, geometry_msgs::Pose trunk_pose);

  void initTrajectories();

};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_
