#include "bitbots_splines/position_spline.hpp"

namespace bitbots_splines {

geometry_msgs::msg::Point PositionSpline::getGeometryMsgPosition(double time) {
  geometry_msgs::msg::Point msg;
  tf2::Vector3 tf_vec = getPos(time);
  msg.x = tf_vec.x();
  msg.y = tf_vec.y();
  msg.z = tf_vec.z();
  return msg;
}

tf2::Vector3 PositionSpline::getPos(double time) { return tf2::Vector3(x_.pos(time), y_.pos(time), z_.pos(time)); }

tf2::Vector3 PositionSpline::getVel(double time) { return tf2::Vector3(x_.vel(time), y_.vel(time), z_.vel(time)); }
tf2::Vector3 PositionSpline::getAcc(double time) { return tf2::Vector3(x_.acc(time), y_.acc(time), z_.acc(time)); }

SmoothSpline *PositionSpline::x() { return &x_; }

SmoothSpline *PositionSpline::y() { return &y_; }

SmoothSpline *PositionSpline::z() { return &z_; }

}  // namespace bitbots_splines
