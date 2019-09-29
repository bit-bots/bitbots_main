#ifndef BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_
#define BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_

#include <string>
#include <optional>
#include <bitbots_splines/SmoothSpline.hpp>
#include <bitbots_splines/SplineContainer.hpp>
#include <bitbots_splines/abstract_engine.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "stabilizer.h"

namespace bitbots_dynup {

struct DynUpParams {
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

class DynupEngine : public bitbots_splines::AbstractEngine<DynupRequest, DynupResponse> {
 public:
  DynupEngine();

  DynupResponse update(double dt) override;

  void setGoals(const DynupRequest &goals) override;

  int getPercentDone() const override;

  void setParams(DynUpParams params);

  void reset() override;
 private:
  double time_;

  std::optional<bitbots_splines::Trajectories> foot_trajectories_, hand_trajectories_, trunk_trajectories_;
  DynUpParams params_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;

  geometry_msgs::PoseStamped getCurrentPose(bitbots_splines::Trajectories spline_container, bool foot);

  void calcFrontSplines();

  void calcBackSplines();

  void calcSquatSplines(geometry_msgs::Pose l_foot_pose, geometry_msgs::Pose trunk_pose);

  void initTrajectories();

};

}

#endif  //BITBOTS_DYNUP_INCLUDE_BITBOTS_DYNUP_DYNUP_ENGINE_H_
