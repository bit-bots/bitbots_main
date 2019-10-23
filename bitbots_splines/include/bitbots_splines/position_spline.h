#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSITION_SPLINE_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSITION_SPLINE_H_

#include <bitbots_splines/SmoothSpline.hpp>
#include <bitbots_splines/SplineContainer.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/Point.h>

namespace bitbots_splines {

class PositionSpline {
 public:
  geometry_msgs::Point getGeometryMsgPosition(double time);

  tf2::Vector3 getPos(double time);
  tf2::Vector3 getVel(double time);
  tf2::Vector3 getAcc(double time);

  SmoothSpline *x();
  SmoothSpline *y();
  SmoothSpline *z();

 private:
  SmoothSpline x_;
  SmoothSpline y_;
  SmoothSpline z_;

};
}
#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSITION_SPLINE_H_
