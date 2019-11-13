/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_SMOOTHSPLINE_HPP_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_SMOOTHSPLINE_HPP_

#include "Spline.hpp"

namespace bitbots_splines {

/**
 * SmoothSpline
 *
 * Implementation of 5th order polynomial
 * splines trajectory known to minimize jerk
 */
class SmoothSpline : public Spline {
 public:

  /**
   * Simple point struture
   */
  struct Point {
    double time;
    double position;
    double velocity;
    double acceleration;
  };

  /**
   * Add a new point with its time, position value,
   * velocity and acceleration
   */
  void addPoint(double time, double position,
                double velocity = 0.0, double acceleration = 0.0);

  /**
   * Access to points container
   */
  const std::vector<Point> &points() const;
  std::vector<Point> &points();

  /**
   * Recompute splines interpolation model
   */
  void computeSplines();

  /**
   * Returns a string representation of the Spline to get inside while debugging.
   * @return
   */
  std::string getDebugString();

 protected:

  /**
   * Inherit
   * Load Points
   */
  void importCallBack() override;

 private:

  /**
   * Points container
   */
  std::vector<Point> points_;

  /**
   * Fit a polynom between 0 and t with given
   * pos, vel and acc initial and final conditions
   */
  Polynom polynomFit(double t,
                     double pos_1, double vel_1, double acc_1,
                     double pos_2, double vel_2, double acc_2) const;
};

}

#endif

