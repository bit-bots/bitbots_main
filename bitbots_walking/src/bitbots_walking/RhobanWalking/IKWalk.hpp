#ifndef LEPH_IKWALK_HPP
#define LEPH_IKWALK_HPP

#include "HumanoidModel.hpp"

namespace Rhoban {
        
/**
 * Walk parameters
 */
struct IKWalkParameters {
    /**
     * Model leg typical length between
     * each rotation axis
     */
    double distHipToKnee;
    double distKneeToAnkle;
    double distAnkleToGround;
    /**
     * Distance between the two feet in lateral
     * axis while in zero position
     */
    double distFeetLateral;
    /**
     * Complete (two legs) walk cycle frequency
     * in Hertz
     */
    double freq;
    /**
     * Global gain multiplying all time
     * dependant movement between 0 and 1.
     * Control walk enabled/disabled smoothing.
     * 0 is walk disabled.
     * 1 is walk fully enabled
     */
    double enabledGain;
    /**
     * Length of double support phase
     * in phase time
     * (between 0 and 1)
     * 0 is null double support and full single support
     * 1 is full double support and null single support
     */
    double supportPhaseRatio;
    /**
     * Lateral offset on default foot 
     * position in meters (foot lateral distance)
     * 0 is default
     * > 0 is both feet external offset
     */
    double footYOffset;
    /**
     * Forward length of each foot step
     * in meters
     * >0 goes forward
     * <0 goes backward
     * (dynamic parameter)
     */
    double stepGain;
    /**
     * Vertical rise height of each foot
     * in meters (positive)
     */
    double riseGain;
    /**
     * Angular yaw rotation of each 
     * foot for each step in radian.
     * 0 does not turn
     * >0 turns left
     * <0 turns right
     * (dynamic parameter)
     */
    double turnGain;
    /**
     * Lateral length of each foot step
     * in meters.
     * >0 goes left
     * <0 goes right
     * (dynamic parameter)
     */
    double lateralGain;
    /**
     * Vertical foot offset from trunk 
     * in meters (positive)
     * 0 is in init position
     * > 0 set the robot lower to the ground
     */
    double trunkZOffset;
    /**
     * Lateral trunk oscillation amplitude
     * in meters (positive)
     */
    double swingGain;
    /**
     * Lateral angular oscillation amplitude
     * of swing trunkRoll in radian
     */
    double swingRollGain;
    /**
     * Phase shift of lateral trunk oscillation
     * between 0 and 1
     */
    double swingPhase;
    /**
     * Foot X-Z spline velocities
     * at ground take off and ground landing.
     * Step stands for X and rise stands for Z
     * velocities.
     * Typical values ranges within 0 and 5.
     * >0 for DownVel is having the foot touching the
     * ground with backward velocity.
     * >0 for UpVel is having the foot going back
     * forward with non perpendicular tangent.
     */
    double stepUpVel;
    double stepDownVel;
    double riseUpVel;
    double riseDownVel;
    /**
     * Time length in phase time
     * where swing lateral oscillation
     * remains on the same side
     * between 0 and 0.5
     */
    double swingPause;
    /**
     * Swing lateral spline velocity (positive).
     * Control the "smoothness" of swing trajectory.
     * Typical values are between 0 and 5.
     */
    double swingVel;
    /**
     * Forward trunk-foot offset 
     * with respect to foot in meters
     * >0 moves the trunk forward
     * <0 moves the trunk backward
     */
    double trunkXOffset;
    /**
     * Lateral trunk-foot offset
     * with respect to foot in meters
     * >0 moves the trunk on the left
     * <0 moves the trunk on the right
     */
    double trunkYOffset;
    /**
     * Trunk angular rotation
     * around Y in radian
     * >0 bends the trunk forward
     * <0 bends the trunk backward
     */
    double trunkPitch;
    /**
     * Trunk angular rotation
     * around X in radian
     * >0 bends the trunk on the right
     * <0 bends the trunk on the left
     */
    double trunkRoll;
    /**
     * Add extra offset on X, Y and Z
     * direction on left and right feet
     * in meters
     */
    double extraLeftX;
    double extraLeftY;
    double extraLeftZ;
    double extraRightX;
    double extraRightY;
    double extraRightZ;
    /**
     * Add extra angular offset on
     * Yaw, Pitch and Roll rotation of 
     * left and right foot in radians
     */
    double extraLeftYaw;
    double extraLeftPitch;
    double extraLeftRoll;
    double extraRightYaw;
    double extraRightPitch;
    double extraRightRoll;
};

/**
 * IKWalk
 *
 * Open loop walk engine based on
 * cubic splines in Cartesian space and
 * inverse kinematics for "standard"
 * small humanoid robots.
 */
class IKWalk
{
    public:

        /**
         * Compute and return target motor reference
         * positions using given walk parameters at given
         * phase (between 0 and 1).
         * Phase is updated according to frequency 
         * parameter and given time step dt.
         * If inverse kinematics fail, false is return and
         * neither phase or output is updated.
         */
        static bool walk(
            const IKWalkParameters& params, 
            double dt,
            double& phase, 
            IKWalkOutputs& outputs);

    private:

        /**
         * Cycle given phase between 0 and 1
         */
        static void boundPhase(double& phase);
};

}

#endif

