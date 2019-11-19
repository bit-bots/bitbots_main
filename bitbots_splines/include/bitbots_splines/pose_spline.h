#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSE_SPLINE_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSE_SPLINE_H_

#include <bitbots_splines/smooth_spline.h>
#include <bitbots_splines/spline_container.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace bitbots_splines {

class PoseSpline {
 public:
  tf2::Transform getTfTransform(double time);

  geometry_msgs::Pose getGeometryMsgPose(double time);
  geometry_msgs::Point getGeometryMsgPosition(double time);
  geometry_msgs::Quaternion getGeometryMsgOrientation(double time);

  tf2::Vector3 getPositionPos(double time);
  tf2::Vector3 getPositionVel(double time);
  tf2::Vector3 getPositionAcc(double time);

  tf2::Vector3 getEulerAngles(double time);
  tf2::Vector3 getEulerVel(double time);
  tf2::Vector3 getEulerAcc(double time);

  tf2::Quaternion getOrientation(double time);

  std::string getDebugString();

  smooth_spline *x();
  smooth_spline *y();
  smooth_spline *z();
  smooth_spline *roll();
  smooth_spline *pitch();
  smooth_spline *yaw();

 private:
  smooth_spline x_;
  smooth_spline y_;
  smooth_spline z_;
  smooth_spline roll_;
  smooth_spline pitch_;
  smooth_spline yaw_;

};
}
#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSE_SPLINE_H_
