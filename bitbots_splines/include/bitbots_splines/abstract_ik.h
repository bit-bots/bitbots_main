
#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_IK_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_IK_H_

#include <vector>
#include <string>
#include <moveit/robot_model/robot_model.h>

namespace bitbots_splines {

    typedef std::pair<std::vector<std::string>, std::vector<double>> JointGoals;

/**
 * Update joint goal values in bulk
 *
 * @param goals Initial goal values
 * @param names List of names which include the joints that should get updated
 * @param values New values for the previously defined joint names
 * @return new joint goals with updated values
 */
    inline JointGoals joint_goals_update(const JointGoals &goals,
                                         const std::vector<std::string> &names,
                                         const std::vector<double> &values) {
        JointGoals result = goals;

        if (names.size() != values.size()) {
            ROS_ERROR("joint_goals_update() called with unequal argument sizes");
        }

        for (int i = 0; i < goals.first.size(); ++i) {
            for (int j = 0; j < names.size() && j < values.size(); ++j) {
                if (result.first.at(i) == names.at(j)) {
                    result.second.at(i) = values.at(j);
                }
            }
        }

        return result;
    }

/**
 * Update joint goal values in bulk by adding the corresponding difference to the current value
 *
 * @param goals Initial goal values
 * @param names List of names which include the joints that should get updated
 * @param diffs Differences to be applied to the previously defined joint names
 * @return new joint goals with updated values
 */
    inline JointGoals joint_goals_update_diff(const JointGoals &goals,
                                              const std::vector<std::string> &names,
                                              const std::vector<double> &diffs) {
        JointGoals result = goals;

        if (names.size() != diffs.size()) {
            ROS_ERROR("joint_goals_update() called with unequal argument sizes");
        }

        for (int i = 0; i < goals.first.size(); ++i) {
            for (int j = 0; j < names.size() && j < diffs.size(); ++j) {
                if (result.first.at(i) == names.at(j)) {
                    result.second.at(i) += diffs.at(j);
                }
            }
        }

        return result;
    }

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