#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_STABILIZER_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_STABILIZER_H_

#include <rclcpp/rclcpp.hpp>

namespace bitbots_splines {

template <typename Positions>
class AbstractStabilizer {
 public:
  /**
   * Reset the stabilizer to its initial state.
   */
  virtual void reset() = 0;
  /**
   * Get a set inverse kinematics goals from cartesian positions. The inverse kinematics goals may contain
   * additional goals, especially for stabilizing.
   * @param positions An instance of Positions that contains the results of the engine's calculations.
   * @return A pointer to BioIK Goals that can be passed to the AbstractIK.
   */
  virtual Positions stabilize(const Positions &positions, const rclcpp::Duration &dt) = 0;
};
}  // namespace bitbots_splines

#endif  // BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_STABILIZER_H_
