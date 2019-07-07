/*
This code is based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef FOOTSTEP_HPP
#define FOOTSTEP_HPP

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include "bitbots_splines/Angle.h"

namespace bitbots_quintic_walk {

/**
 * Footstep
 *
 * Manage humanoid footstep
 * generation and state
 */
class Footstep
{
    public:

        enum SupportFoot {
            LeftSupportFoot,
            RightSupportFoot,
        };

        /**
         * Initialization with lateral
         * foot distance and support foot
         */
        Footstep(
            double footDistance,
            bool isLeftSupportFoot = true);

        /**
         * Set the lateral foot 
         * distance parameters
         */
        void setFootDistance(double footDistance);
        double getFootDistance();

        /**
         * Reset to neutral position the current
         * step (not the integrated odometry)
         */
        void reset(bool isLeftSupportFoot);

        // reset odometry
        void resetInWorld(bool isLeftSupportFoot);

        /**
         * Current support foot
         */
        bool isLeftSupport() const;

        /**
         * Starting position of current flying 
         * foot in support foot frame
         */
        const Eigen::Vector3d& getLast() const;

        /**
         * Target pose of current flying 
         * foot in support foot frame
         */
        const Eigen::Vector3d& getNext() const;

        /**
         * Returns the odometry change of the current step.
         */ 
        const Eigen::Vector3d& getOdom() const;

        /**
         * Left and right, current or next pose
         * of foot in world initial frame
         */
        const Eigen::Vector3d& getLeft() const;
        const Eigen::Vector3d& getRight() const;

        /**
         * Set the target pose of current support foot
         * during next support phase and update support foot.
         * The target foot pose diff is given with respect to
         * next support foot pose (current flying foot target).
         */
        void stepFromSupport(const Eigen::Vector3d& diff);

        /**
         * Set target pose of current support foot
         * using diff orders. 
         * Zero vector means in place walking.
         * Special handle of lateral and turn step
         * to avoid foot collision.
         */
        void stepFromOrders(const Eigen::Vector3d& diff);

    private:

        /**
         * Static lateral distance 
         * between the feet
         */
        double _footDistance;

        /**
         * Current support foot
         * (left or right)
         */
        bool _isLeftSupportFoot;

        /**
         * Pose diff [dx, dy, dtheta]
         * from support foot to flying foot
         * last and next position
         */
        Eigen::Vector3d _supportToLast;
        Eigen::Vector3d _supportToNext;

        /**
         * Pose integration of left
         * and right foot in initial frame.
         * Set at "future" state taking into account
         * next expected fot pose.
         */
        Eigen::Vector3d _leftInWorld;
        Eigen::Vector3d _rightInWorld;

        /**
         * Add to given pose the given diff 
         * expressed in pose frame and
         * return the integrated added pose
         */
        Eigen::Vector3d poseAdd(
            const Eigen::Vector3d& pose,
            const Eigen::Vector3d& diff) const;

        /**
         * Compute and return the delta from
         * (zero+diff) to (zero) in 
         * (zero+diff) frame.
         */
        Eigen::Vector3d diffInv(
            const Eigen::Vector3d& diff) const;
};

}
#endif