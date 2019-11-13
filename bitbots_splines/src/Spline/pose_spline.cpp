#include "bitbots_splines/pose_spline.h"

namespace bitbots_splines {

tf2::Transform PoseSpline::getTfTransform(double time) {
  tf2::Transform trans;
  trans.setOrigin(getPositionPos(time));
  trans.setRotation(getOrientation(time));
  return trans;
}
geometry_msgs::Pose PoseSpline::getGeometryMsgPose(double time) {
  geometry_msgs::Pose msg;
  msg.position = getGeometryMsgPosition(time);
  msg.orientation = getGeometryMsgOrientation(time);
  return msg;
}

geometry_msgs::Point PoseSpline::getGeometryMsgPosition(double time) {
  geometry_msgs::Point msg;
  tf2::Vector3 tf_vec = getPositionPos(time);
  msg.x = tf_vec.x();
  msg.y = tf_vec.y();
  msg.z = tf_vec.z();
  return msg;
}

geometry_msgs::Quaternion PoseSpline::getGeometryMsgOrientation(double time) {
  geometry_msgs::Quaternion msg;
  tf2::convert(getOrientation(time), msg);
  return msg;
}

tf2::Vector3 PoseSpline::getPositionPos(double time) {
  tf2::Vector3 pos;
  pos[0] = x_.pos(time);
  pos[1] = y_.pos(time);
  pos[2] = z_.pos(time);
  return pos;
}

tf2::Vector3 PoseSpline::getPositionVel(double time) {
  tf2::Vector3 vel;
  vel[0] = x_.vel(time);
  vel[1] = y_.vel(time);
  vel[2] = z_.vel(time);
  return vel;
}
tf2::Vector3 PoseSpline::getPositionAcc(double time) {
  tf2::Vector3 acc;
  acc[0] = x_.acc(time);
  acc[1] = y_.acc(time);
  acc[2] = z_.acc(time);
  return acc;
}

tf2::Vector3 PoseSpline::getEulerAngles(double time) {
  tf2::Vector3 pos;
  pos[0] = roll_.pos(time);
  pos[1] = pitch_.pos(time);
  pos[2] = yaw_.pos(time);
  return pos;
}
tf2::Vector3 PoseSpline::getEulerVel(double time) {
  tf2::Vector3 vel;
  vel[0] = roll_.vel(time);
  vel[1] = pitch_.vel(time);
  vel[2] = yaw_.vel(time);
  return vel;
}
tf2::Vector3 PoseSpline::getEulerAcc(double time) {
  tf2::Vector3 acc;
  acc[0] = roll_.acc(time);
  acc[1] = pitch_.acc(time);
  acc[2] = yaw_.acc(time);
  return acc;
}

tf2::Quaternion PoseSpline::getOrientation(double time) {
  tf2::Quaternion quat;
  tf2::Vector3 rpy = getEulerAngles(time);
  quat.setRPY(rpy[0], rpy[1], rpy[2]);
  quat.normalize();
  return quat;
}

smooth_spline *PoseSpline::x() {
  return &x_;
}

smooth_spline *PoseSpline::y() {
  return &y_;
}

smooth_spline *PoseSpline::z() {
  return &z_;
}

smooth_spline *PoseSpline::roll() {
  return &roll_;
}

smooth_spline *PoseSpline::pitch() {
  return &pitch_;
}

smooth_spline *PoseSpline::yaw() {
  return &yaw_;
}

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

}