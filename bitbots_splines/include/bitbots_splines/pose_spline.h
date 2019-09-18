#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSE_SPLINE_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSE_SPLINE_H_

#include <bitbots_splines/SmoothSpline.hpp>
#include <bitbots_splines/SplineContainer.hpp>
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

  SmoothSpline x();
  SmoothSpline y();
  SmoothSpline z();
  SmoothSpline roll();
  SmoothSpline pitch();
  SmoothSpline yaw();

 private:
  SmoothSpline x_;
  SmoothSpline y_;
  SmoothSpline z_;
  SmoothSpline roll_;
  SmoothSpline pitch_;
  SmoothSpline yaw_;

};
}
#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSE_SPLINE_H_
