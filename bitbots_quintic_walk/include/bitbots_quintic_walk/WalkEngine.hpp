/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef QUINTICWALK_HPP
#define QUINTICWALK_HPP

#include <algorithm> 
#include <Eigen/Dense>
#include "Footstep.hpp"
#include "bitbots_splines/TrajectoryUtils.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include "bitbots_splines/SplineContainer.hpp"
#include "bitbots_splines/AxisAngle.h"
#include "bitbots_splines/Angle.h"
#include "bitbots_splines/Euler.h"
#include "bitbots_quintic_walk/common.h"
#include <math.h>       

namespace bitbots_quintic_walk {

struct WalkingParameter{
    //Full walk cycle frequency 
    //(in Hz, > 0)
    double freq;
    //Length of double support phase in half cycle
    //(ratio, [0:1])
    double doubleSupportRatio;
    //Lateral distance between the feet center 
    //(in m, >= 0)
    double footDistance;
    //Maximum flying foot height
    //(in m, >= 0)
    double footRise;
    // Pause of Z movement on highest point
    //(single support cycle ratio, [0,1]) 
    double footZPause;
    //Let the foot's downward trajectory end above the ground
    //this is helpful if the support leg bends
    //(in m, >= 0)
    double footPutDownZOffset;
    //Phase time for moving the foot from Z offset to ground, 
    // also used for X and Y since they should not move after contact to the ground
    //(phase between apex and single support end [0:1])
    double footPutDownPhase;
    //Phase of flying foot apex
    //(single support cycle phase, [0:1])
    double footApexPhase;
    //Foot X/Y overshoot in ratio of step length
    //(ratio, >= 0)
    double footOvershootRatio;
    //Foot X/Y overshoot phase
    //(single support cycle phase, [footApexPhase:1]
    double footOvershootPhase;
    //Height of the trunk from ground
    //(in m, > 0)
    double trunkHeight;
    //Trunk pitch orientation
    //(in rad)
    double trunkPitch;
    //Phase offset of trunk oscillation
    //(half cycle phase, [0:1])
    double trunkPhase;
    //Trunk forward offset
    //(in m)
    double trunkXOffset;
    //Trunk lateral offset
    //(in m)
    double trunkYOffset;
    //Trunk lateral oscillation amplitude ratio
    //(ratio, >= 0)
    double trunkSwing;
    //Trunk swing pause length in phase at apex
    //(half cycle ratio, [0:1])
    double trunkPause;
    //Trunk forward offset proportional to forward step
    //(in 1)
    double trunkXOffsetPCoefForward;
    //Trunk forward offset proportional to rotation step
    //(in m/rad)
    double trunkXOffsetPCoefTurn;
    //Trunk pitch orientation proportional to forward step
    //(in rad/m)
    double trunkPitchPCoefForward;
    //Trunk pitch orientation proportional to rotation step
    //(in 1)
    double trunkPitchPCoefTurn;
    double trunkYOnlyInDoubleSupport;
    double kickLength;
    double kickPhase;
    double footPutDownRollOffset;
    double kickVel;
};

/**
 * QuinticWalk
 *
 * Holonomic and open loop walk 
 * generator based on footstep control 
 * and quintic splines in cartesian space.
 * Expressed all target state in cartesian
 * space with respect to current cupport foot
 */
class QuinticWalk
{
    public:

        /**
         * Initialization
         */
        QuinticWalk();

        /**
         * Return current walk phase
         * between 0 and 1
         */
        double getPhase() const;

        /**
         * Return current time between
         * 0 and half period for 
         * trajectories evaluation
         */
        double getTrajsTime() const;

        /**
         * Get current used 
         * parameters
         */
        const WalkingParameter& getParameters() const;

        /**
         * Get current walk footstep orders
         */
        const Eigen::Vector3d& getOrders() const;

        /**
         * Get the footstep object.
         */
        Footstep getFootstep();

        /**
         * Get the weight balance between left and right leg.
         * 1.0 means all weight on left leg
         * 0.0 means all weight on right leg
         * during double support phase, the value is between those two values
         */
        double getWeightBalance();

        /**
         * Return true is the walk 
         * oscillations are enabled
         */
        bool isEnabled() const;

        /**
         * Return if true if left is current support foot
         */
        bool isLeftSupport();

        /**
         * Return true if both feet are currently on the ground
         */ 
        bool isDoubleSupport();

        /**
         * Assign given parameters vector
         */
        void setParameters(const WalkingParameter& params);

        /**
         * Rebuilt the trajectories and
         * reset saved state as disable.
         * Used to directly apply 
         * newly parameters.
         */
        void forceRebuildTrajectories();

        /**
         * Set used walk footstep orders,
         * enable or disable the walk oscillations
         * and optionnaly set the starting 
         * supporting foot.
         */
        void setOrders(
            const Eigen::Vector3d& orders, 
            bool isEnabled,
            bool instantStop);

        /**
         * Return the trajectories for
         * current half cycle
         */
        const bitbots_splines::Trajectories& getTrajectories() const;

        /**
         * Update the internal walk state
         * (pĥase, trajectories) from given 
         * elapsed time since last update() call
         */
        void update(double dt);

        /**
         * Compute current cartesian
         * target from trajectories and assign
         * it to given model through inverse
         * kinematics.
         * Return false is the target is
         * unreachable.
         */

        void computeCartesianPosition(Eigen::Vector3d& trunkPos, Eigen::Vector3d& trunkAxis,
                                                   Eigen::Vector3d& footPos, Eigen::Vector3d& footAxis, bool& isLeftsupportFoot);

        void computeCartesianPositionAtTime(Eigen::Vector3d& trunkPos, Eigen::Vector3d& trunkAxis, Eigen::Vector3d& footPos,
                                        Eigen::Vector3d& footAxis, bool& isLeftsupportFoot, double time);

        void saveSplineCsv(const std::string& filename);

        void setKick(bool left);

        bool getIsWalking();

        void resetPhase();

private:

        /**
         * Current footstep support
         * and flying last and next pose
         */
        Footstep _footstep;

        /**
         * Movement phase between 0 and 1
         */
        double _phase;

        /**
         * Currently used parameters
         */
        WalkingParameter _params;

        /**
         * Currently used footstep
         * orders flush at next suppot 
         * foot swap
         */
        Eigen::Vector3d _orders;

        /**
         * Enable or disable
         * the oscillations and
         * value at last half cycle.
         */
        bool _isEnabled;
        bool _wasEnabled;

        bool _isLeftKick;
        bool _isRightKick;
        
        /**
         * Trunk pose and orientation 
         * position, velocity and acceleration 
         * at half cycle start
         */
        Eigen::Vector3d _trunkPosAtLast;
        Eigen::Vector3d _trunkVelAtLast;
        Eigen::Vector3d _trunkAccAtLast;
        Eigen::Vector3d _trunkAxisPosAtLast;
        Eigen::Vector3d _trunkAxisVelAtLast;
        Eigen::Vector3d _trunkAxisAccAtLast;

        /**
         * Generated half walk 
         * cycle trajectory
         */
        bitbots_splines::Trajectories _trajs;

        /**
         * Reset and rebuild the
         * spline trajectories for
         * current half cycle
         */
        void buildTrajectories();

        bool _isWalking;
        bool _instantStop;
        bool _doCaptureStep;
        bool _didDisableStep;
        void saveCurrentTrunkState();
        void useCurrentTrunkState();
        void buildInstantStopTrajectories();
        void buildCaptureStepTrajectories();
        void buildWalkDisableTrajectories();

        void point(std::string spline, double t, double pos, double vel = 0, double acc = 0);
        
        /**
         * Reset the trunk position and
         * orientation state vectors at last
         * half cycle as stopped pose
         */
        void resetTrunkLastState();
};

}
#endif