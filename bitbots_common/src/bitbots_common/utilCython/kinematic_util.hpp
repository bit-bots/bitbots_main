#ifndef _KINEMATIC_UTIL_HPP__
#define _KINEMATIC_UTIL_HPP__

#include <Eigen/Core>

#include "../robot/forward.hpp"
#include "../robot/jointids.hpp"
#include "../../../../bitbots_motion/src/bitbots_motion/walking/zmp_math_basics.h"  //TODO: Diese abhängikkeit ist mist (was hat die kinematik mit dem walking zu schaffen?)
#include "eigen_util.hpp"

namespace Util{
namespace Kinematics {

using Robot::Kinematics::KRobot;
using Robot::Kinematics::JointIds;

using ZMPFootPhaseDefinition::FootPhase;
enum SupportLeg{unknown=FootPhase::unknown, both=FootPhase::both, left=FootPhase::left, right=FootPhase::right};

/**
 * \brief This is the kinematic util, providing some kinematic tasks that could be interesting
 *
 * This util class only provides static methods.
 * You can calculate head angles to focus a certain point, or try the experimental method to keep the robot statically stable.
 * More methods will follow when they are considered helpfully.
 */
class KinematicUtil {
private:

    template<int additional_parameter>
    static inline void get_head_angles_for_distance_intern(KRobot& robot, const Eigen::Vector2d& pos, bool left_leg, double y_angle=0, double x_angle=0);

public:

    /**
     * This method is able to provide an angular, representing the way the robot is looking at the world.
     * On the first component, there is the angular in rad, of the position of the centred horizon line according to the camera position.
     * The second component represents the robots roll angle according to the camera.
     * \param robot: The robot used for calculation
     * \param phase: The foot phase used as basis to determine the support foot. It's assumed the support foot stands on even ground
     * \param rpy: The robots rotations measured by the robots sensors to give the model initial angles.
     */
    static Eigen::Vector2d get_robot_horizon(const KRobot& robot, uint8_t phase);
    static Eigen::Vector2d get_robot_horizon(KRobot& robot, const Eigen::Vector3d& rpy);

    static Eigen::Vector3d get_camera_position(const KRobot& robot, uint8_t phase);

    /**
     * This Method calculates joint angles for a special kinematic task.
     * \param pos: The point that will be focussed anyhow
     * \param left_leg: true if the robot's support leg is the left, false otherwise
     * \param y_angle: the in picture angle on the y-axis of the point, that will have the given distance:(range -0.5, 0.5)
     * \param x_angle: the in picture angle on the x-axis of the point, that will have the given distance:(range ~-0.3, ~0.3)
     */
    static void get_head_angles_for_distance(KRobot& robot, const Eigen::Vector2d& pos, bool left_leg);
    static void get_head_angles_for_distance(KRobot& robot, const Eigen::Vector2d& pos, bool left_leg, double y_angle);
    static void get_head_angles_for_distance(KRobot& robot, const Eigen::Vector2d& pos, bool left_leg, double y_angle, double x_angle);

    /**
     * Determines the support leg using the feet relative positions
     * \param robot: The robot used to calculate the leg
     * \param threshold: a threshold to allow a state with both feet on the ground
     */
    static SupportLeg determine_support_leg(const KRobot& robot, const double threshold=1);

    static Eigen::Vector2d calculate_kinematic_robot_angle(const KRobot& robot, int sup);
};

} } // namespace Util::Kinematics

#endif //_KINEMATIC_UTIL_HPP__
