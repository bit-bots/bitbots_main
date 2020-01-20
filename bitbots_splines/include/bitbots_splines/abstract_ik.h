#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_IK_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_IK_H_

#include <vector>
#include <string>
#include <moveit/robot_model/robot_model.h>

namespace bitbots_splines {

typedef std::pair<std::vector<std::string>, std::vector<double>> JointGoals;

template<typename Positions>
class AbstractIK {
  /**
   * Initializes the class. This must be called before calculate() is called.
   * @param kinematic_model The MoveIt! kinematic model of the robot
   */
  virtual void init(moveit::core::RobotModelPtr kinematic_model) = 0;
  /**
   * Calculate motor joint goals from cartesian positions, i.e. solve the presented inverse kinematics problem
   * @param positions the cartesian positions of end points and custom additional information (e.g. support foot)
   * @return motor positions
   */
  virtual JointGoals calculate(const Positions &positions) = 0;
  /**
   * Reset the IK to its initial state.
   */
  virtual void reset() = 0;
};
}

#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_IK_H_
