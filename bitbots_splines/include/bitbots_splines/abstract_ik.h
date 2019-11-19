#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_IK_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_IK_H_

#include <vector>
#include <string>
#include <bio_ik/bio_ik.h>

namespace bitbots_splines {

typedef std::pair<std::vector<std::string>, std::vector<double>> JointGoals;

class AbstractIK {
  /**
   * Initializes the class. This must be called before calculate() is called.
   * @param kinematic_model The MoveIt! kinematic model of the robot
   */
  virtual void init(moveit::core::RobotModelPtr kinematic_model) = 0;
  /**
   * Calculate motor joint goals from IK goals, i.e. solve the presented inverse kinematics problem
   * @param ik_goals
   * @return
   */
  virtual JointGoals calculate(std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) = 0;
  /**
   * Reset the IK to its initial state.
   */
  virtual void reset() = 0;
};
}

#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_IK_H_
