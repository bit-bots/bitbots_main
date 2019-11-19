#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSITION_SPLINE_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSITION_SPLINE_H_

#include <bitbots_splines/smooth_spline.h>
#include <bitbots_splines/spline_container.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/Point.h>

namespace bitbots_splines {

class PositionSpline {
 public:
  geometry_msgs::Point getGeometryMsgPosition(double time);

  tf2::Vector3 getPos(double time);
  tf2::Vector3 getVel(double time);
  tf2::Vector3 getAcc(double time);

  smooth_spline *x();
  smooth_spline *y();
  smooth_spline *z();

 private:
  smooth_spline x_;
  smooth_spline y_;
  smooth_spline z_;

};
}
#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSITION_SPLINE_H_
