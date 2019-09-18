#include "bitbots_splines/position_spline.h"

namespace bitbots_splines {


geometry_msgs::Point PositionSpline::getGeometryMsgPosition(double time) {
  geometry_msgs::Point msg;
  tf2::Vector3 tf_vec = getPos(time);
  msg.x = tf_vec.x();
  msg.y = tf_vec.y();
  msg.z = tf_vec.z();
  return msg;
}

tf2::Vector3 PositionSpline::getPos(double time) {
  tf2::Vector3 pos;
  pos[0] = x_.pos(time);
  pos[1] = y_.pos(time);
  pos[2] = z_.pos(time);
  return pos;
}

tf2::Vector3 PositionSpline::getVel(double time) {
  tf2::Vector3 vel;
  vel[0] = x_.vel(time);
  vel[1] = y_.vel(time);
  vel[2] = z_.vel(time);
  return vel;
}
tf2::Vector3 PositionSpline::getAcc(double time){
  tf2::Vector3 acc;
  acc[0] = x_.acc(time);
  acc[1] = y_.acc(time);
  acc[2] = z_.acc(time);
  return acc;
}


SmoothSpline PositionSpline::getSplineX(){
  return x_;
}

SmoothSpline PositionSpline::getSplineY(){
  return y_;
}

SmoothSpline PositionSpline::getSplineZ(){
  return z_;
}

}
