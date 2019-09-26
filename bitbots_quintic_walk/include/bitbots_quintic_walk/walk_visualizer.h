#ifndef BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_VISUALIZER_H_
#define BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_VISUALIZER_H_

#include <ros/ros.h>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "bitbots_splines/abstract_visualizer.h"
#include "bitbots_splines/abstract_ik.h"
#include <bitbots_quintic_walk/WalkDebug.h>
#include <bitbots_quintic_walk/WalkEngineDebug.h>
#include <moveit_msgs/RobotState.h>
#include <bitbots_quintic_walk/walk_utils.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace bitbots_quintic_walk {
class WalkVisualizer : public bitbots_splines::AbstractVisualizer {
 public:
  explicit WalkVisualizer();

  void publishArrowMarker(std::string name_space,
                          std::string frame,
                          geometry_msgs::Pose pose,
                          float r,
                          float g,
                          float b,
                          float a);

  void publishEngineDebug(WalkResponse response);
  void publishIKDebug(WalkResponse response,
                      robot_state::RobotStatePtr current_state,
                      bitbots_splines::JointGoals joint_goals);
  void publishWalkMarkers(WalkResponse response);

 private:

  int marker_id_;

  ros::Publisher pub_debug_;
  ros::Publisher pub_engine_debug_;
  ros::Publisher pub_debug_marker_;
};
}

#endif //BITBOTS_QUINTIC_WALK_INCLUDE_BITBOTS_QUINTIC_WALK_WALK_VISUALIZER_H_
