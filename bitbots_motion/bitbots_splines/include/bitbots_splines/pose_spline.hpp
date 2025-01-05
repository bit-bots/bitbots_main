#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSE_SPLINE_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSE_SPLINE_H_

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <bitbots_splines/smooth_spline.hpp>
#include <bitbots_splines/spline_container.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bitbots_splines {

class PoseSpline {
 public:
  tf2::Transform getTfTransform(double time);

  geometry_msgs::msg::Pose getGeometryMsgPose(double time);
  geometry_msgs::msg::Point getGeometryMsgPosition(double time);
  geometry_msgs::msg::Quaternion getGeometryMsgOrientation(double time);

  tf2::Vector3 getPositionPos(double time);
  tf2::Vector3 getPositionVel(double time);
  tf2::Vector3 getPositionAcc(double time);

  tf2::Vector3 getEulerAngles(double time);
  tf2::Vector3 getEulerVel(double time);
  tf2::Vector3 getEulerAcc(double time);

  tf2::Quaternion getOrientation(double time);

  std::string getDebugString();

  SmoothSpline *x();
  SmoothSpline *y();
  SmoothSpline *z();
  SmoothSpline *roll();
  SmoothSpline *pitch();
  SmoothSpline *yaw();

 private:
  SmoothSpline x_;
  SmoothSpline y_;
  SmoothSpline z_;
  SmoothSpline roll_;
  SmoothSpline pitch_;
  SmoothSpline yaw_;
};
}  // namespace bitbots_splines
#endif  // BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSE_SPLINE_H_
