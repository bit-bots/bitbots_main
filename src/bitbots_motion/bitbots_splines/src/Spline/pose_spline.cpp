#include <bitbots_splines/pose_spline.hpp>

namespace bitbots_splines {

tf2::Transform PoseSpline::getTfTransform(double time) {
  tf2::Transform trans;
  trans.setOrigin(getPositionPos(time));
  trans.setRotation(getOrientation(time));
  return trans;
}
geometry_msgs::msg::Pose PoseSpline::getGeometryMsgPose(double time) {
  geometry_msgs::msg::Pose msg;
  msg.position = getGeometryMsgPosition(time);
  msg.orientation = getGeometryMsgOrientation(time);
  return msg;
}

geometry_msgs::msg::Point PoseSpline::getGeometryMsgPosition(double time) {
  geometry_msgs::msg::Point msg;
  tf2::Vector3 tf_vec = getPositionPos(time);
  msg.x = tf_vec.x();
  msg.y = tf_vec.y();
  msg.z = tf_vec.z();
  return msg;
}

geometry_msgs::msg::Quaternion PoseSpline::getGeometryMsgOrientation(double time) {
  geometry_msgs::msg::Quaternion msg;
  tf2::convert(getOrientation(time), msg);
  return msg;
}

tf2::Vector3 PoseSpline::getPositionPos(double time) { return tf2::Vector3(x_.pos(time), y_.pos(time), z_.pos(time)); }

tf2::Vector3 PoseSpline::getPositionVel(double time) { return tf2::Vector3(x_.vel(time), y_.vel(time), z_.vel(time)); }
tf2::Vector3 PoseSpline::getPositionAcc(double time) { return tf2::Vector3(x_.acc(time), y_.acc(time), z_.acc(time)); }

tf2::Vector3 PoseSpline::getEulerAngles(double time) {
  return tf2::Vector3(roll_.pos(time), pitch_.pos(time), yaw_.pos(time));
}
tf2::Vector3 PoseSpline::getEulerVel(double time) {
  return tf2::Vector3(roll_.vel(time), pitch_.vel(time), yaw_.vel(time));
}
tf2::Vector3 PoseSpline::getEulerAcc(double time) {
  return tf2::Vector3(roll_.acc(time), pitch_.acc(time), yaw_.acc(time));
}

tf2::Quaternion PoseSpline::getOrientation(double time) {
  tf2::Quaternion quat;
  tf2::Vector3 rpy = getEulerAngles(time);
  quat.setRPY(rpy[0], rpy[1], rpy[2]);
  quat.normalize();
  return quat;
}

SmoothSpline* PoseSpline::x() { return &x_; }

SmoothSpline* PoseSpline::y() { return &y_; }

SmoothSpline* PoseSpline::z() { return &z_; }

SmoothSpline* PoseSpline::roll() { return &roll_; }

SmoothSpline* PoseSpline::pitch() { return &pitch_; }

SmoothSpline* PoseSpline::yaw() { return &yaw_; }

std::string PoseSpline::getDebugString() {
  std::string output;
  output += "x:\n" + x_.getDebugString() + "\n";
  output += "y:\n" + y_.getDebugString() + "\n";
  output += "z:\n" + z_.getDebugString() + "\n";
  output += "roll:\n" + roll_.getDebugString() + "\n";
  output += "pitch:\n" + pitch_.getDebugString() + "\n";
  output += "yaw:\n" + yaw_.getDebugString() + "\n";
  return output;
}

}  // namespace bitbots_splines
