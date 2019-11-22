#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_ENGINE_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_ENGINE_H_

#include <vector>

namespace bitbots_splines {

template<typename RequestType, typename ResultType>
class AbstractEngine {
 public:
  /**
   * Do one iteration of spline-progress-updating. This means that whenever update() is called,
   *      new position goals are retrieved from previously calculated splines and returned.
   * @param dt Passed time delta between last call to update() and now. Measured in seconds
   * @return New spline positions
   */
  virtual ResultType update(double dt) = 0;
  /**
   * Set new goals for the engine to calculate splines from.
   * @param goals An instance of RequestType that describes the goals that the spline should fulfill
   */
  virtual void setGoals(const RequestType &goals) = 0;
  /**
   * Reset the engine to its initial state.
   */
  virtual void reset() = 0;
  /**
   * Returns the percentage of the spline that has already been returned.
   */
  virtual int getPercentDone() const = 0;
};
}

#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_ENGINE_H_
