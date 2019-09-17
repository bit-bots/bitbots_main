#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACTENGINE_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACTENGINE_H_

#include <vector>
#include <bitbots_splines/SplineContainer.hpp>
#include <bitbots_splines/SmoothSpline.hpp>

namespace bitbots_splines {

typedef bitbots_splines::SplineContainer<bitbots_splines::SmoothSpline> Trajectories;

template<typename Positions, typename Goals>
class AbstractEngine {
 public:
  /**
   * Do one iteration of spline-progress-updating. This means that whenever update() is called,
   *      new position goals are retrieved from previously calculated splines and returned.
   * @param dt Passed time delta between last call to update() and now. Measured in seconds
   * @return New spline positions
   */
  virtual Positions update(double dt) = 0;
  virtual void setGoals(const Goals &goals) = 0;
  virtual void reset() = 0;
  virtual Trajectories getSplines() const = 0;
  /**
   * Returns the percentage of the spline that has already been returned.
   */
  virtual int getPercentDone() const = 0;
};
}

#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACTENGINE_H_
