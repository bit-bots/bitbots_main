#include <iostream>
#include "walking.hpp"
#include "math_constants.h"
#include "cmath"
/**
 * Run the walk for given among of time and update
 * phase and time state
 */



static inline double trim_to_range(double f){
    if(f > 180) {f -= 360 * ((long)((f+180)/360));std::cout<<"Wuahhhh!!!!!!!!!"<<f<<std::endl;}
    //fmod()??
    if(f < -180){f += 360 * ((long)((f-180)/360));std::cout<<"Wuahhhh!!!!!!!!!"<<f<<std::endl;}
    return f;
}



RhobanWalk::RhobanWalk(): params(), outputs()
{
    phase = 0.0;
    /**
    * Model leg typical length between
    * each rotation axis
     **/
   params.distHipToKnee = 0.15;
   params.distKneeToAnkle = 0.15;
   params.distAnkleToGround = 0.04;
   /**
    * Distance between the two feet in lateral
    * axis while in zero position
    */
    params.distFeetLateral = 0.12; //Nur Relevant wen  SwingRollGain != 0
    /**
     * Complete (two legs) walk cycle frequency
     * in Hertz
     */
    params.freq = 2.4;
    /**
     * Global gain multiplying all time
     * dependant movement between 0 and 1.
     * Control walk enabled/disabled smoothing.
     * 0 is walk disabled.
     * 1 is walk fully enabled
     */
    params.enabledGain = 0.0;
    /**
     * Length of double support phase
     * in phase time
     * (between 0 and 1)
     * 0 is null double support and full single support
     * 1 is full double support and null single support
     */
    params.supportPhaseRatio = 0.0;
    /**
     * Lateral offset on default foot
     * position in meters (foot lateral distance)
     * 0 is default
     * > 0 is both feet external offset
     */
    params.footYOffset = 0.025; //Schrittbreite
    /**
     * Forward length of each foot step
     * in meters
     * >0 goes forward
     * <0 goes backward
     * (dynamic parameter)
     */
    params.stepGain = 0.0;
    /**
     * Vertical rise height of each foot
     * in meters (positive)
     */
    params.riseGain = 0.035;
    /**
     * Angular yaw rotation of each
     * foot for each step in radian.
     * 0 does not turn
     * >0 turns left
     * <0 turns right
     * (dynamic parameter)
     */
    params.turnGain = 0.0;
    /**
     * Lateral length of each foot step
     * in meters.
     * >0 goes left
     * <0 goes right
     * (dynamic parameter)
     */
    params.lateralGain = 0.0;
    /**
     * Vertical foot offset from trunk
     * in meters (positive)
     * 0 is in init position
     * > 0 set the robot lower to the ground
     */
    params.trunkZOffset = 0.02; // Höhe des Schwerpunktes relativ zur Initpose
    /**
     * Lateral trunk oscillation amplitude
     * in meters (positive)
     */
    params.swingGain = 0.02; //Y-Ausgleichbewegung des Oberkörpers
    /**
     * Lateral angular oscillation amplitude
     * of swing trunkRoll in radian
     */
    params.swingRollGain = 0.0;
    /**
     * Phase shift of lateral trunk oscillation
     * between 0 and 1
     */
    params.swingPhase = 0.25;
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
    params.stepUpVel = 4.0;
    params.stepDownVel = 4.0;
    params.riseUpVel = 4.0;
    params.riseDownVel = 4.0;
    /**
     * Time length in phase time
     * where swing lateral oscillation
     * remains on the same side
     * between 0 and 0.5
     */
    params.swingPause = 0.0;
    /**
     * Swing lateral spline velocity (positive).
     * Control the "smoothness" of swing trajectory.
     * Typical values are between 0 and 5.
     */
    params.swingVel = 4.0;
    /**
     * Forward trunk-foot offset
     * with respect to foot in meters
     * >0 moves the trunk forward
     * <0 moves the trunk backward
     */
    params.trunkXOffset = 0.019;
    /**
     * Lateral trunk-foot offset
     * with respect to foot in meters
     * >0 moves the trunk on the left
     * <0 moves the trunk on the right
     */
    params.trunkYOffset = 0.0;
    /**
     * Trunk angular rotation
     * around Y in radian
     * >0 bends the trunk forward
     * <0 bends the trunk backward
     */
    params.trunkPitch = 0.15;
    /**
     * Trunk angular rotation
     * around X in radian
     * >0 bends the trunk on the right
     * <0 bends the trunk on the left
     */
    params.trunkRoll = 0.0;
    /**
     * Add extra offset on X, Y and Z
     * direction on left and right feet
     * in meters
     * (Can be used for example to implement
     * dynamic kick)
     */
    params.extraLeftX = 0.0;
    params.extraLeftY = 0.0;
    params.extraLeftZ = 0.0;
    params.extraRightX = 0.0;
    params.extraRightY = 0.0;
    params.extraRightZ = 0.0;
    /**
     * Add extra angular offset on
     * Yaw, Pitch and Roll rotation of
     * left and right foot in radians
     */
    params.extraLeftYaw = 0.0;
    params.extraLeftPitch = 0.0;
    params.extraLeftRoll = 0.0;
    params.extraRightYaw = 0.0;
    params.extraRightPitch = 0.0;
    params.extraRightRoll = 0.0;

}

void RhobanWalk::set_velocity(double vx, double vy, double va)
{
    goalValX = vx;
    goalValY = vy;
    goalValA = va;
}

void RhobanWalk::updateVelocity()
{
    params.stepGain += (goalValX - params.stepGain)/5;
    params.lateralGain += (goalValY - params.lateralGain)/5;
    params.turnGain += (goalValA - params.turnGain)/5;
}

void RhobanWalk::start(){
    params.enabledGain = 1.0;
    stopRequest = 0;
    if (!m_active)
    params.trunkXOffset = trunkXOffset_backUp;
    dynamicTrunkOffset = 0;
    m_active = true;
    }

void RhobanWalk::updateGyro()
{
    if (gyroY > 5)
    {
        dynamicTrunkOffset -= 0.003;
        dynamicTrunkOffset = fmin(fmax(dynamicTrunkOffset, -0.015), 0.015);
        params.trunkXOffset = trunkXOffset_backUp + dynamicTrunkOffset;
        std::cout << "Gyro" << gyroY << " dynamicTrunk" << dynamicTrunkOffset << std::endl;
    }
    else
    {
    std::cout << "Gyro " << gyroY << std::endl;
    }
}

void RhobanWalk::set_frequenzy(double freq)
    {
    //engineFrequenzy = (int)freq;
    }


ZMPFootPhaseDefinition::FootPhase RhobanWalk::update()
{
    bool success = Rhoban::IKWalk::walk(
            params, //Walk parameters
            1.0/engineFrequenzy, //Time step
            phase, //Current walk phase -will be updated)
            outputs); //Result target position (updated)
    if (!success) {
        //The requested position for left or right foot is not feasible
        //(phase is not updated)
        std::cout << "Walk: " << time << " Inverse Kinematics error. Position not reachable."<< std::endl ;
    } else {
        m_pose.get_l_hip_yaw().set_goal(trim_to_range(outputs.left_hip_yaw  * rad_to_degree));
        m_pose.get_l_hip_roll().set_goal(-trim_to_range(outputs.left_hip_roll * rad_to_degree));
        m_pose.get_l_hip_pitch().set_goal(-trim_to_range(outputs.left_hip_pitch* rad_to_degree));
        m_pose.get_l_knee().set_goal(-trim_to_range(outputs.left_knee * rad_to_degree));
        m_pose.get_l_ankle_pitch().set_goal(trim_to_range(outputs.left_ankle_pitch* rad_to_degree));
        m_pose.get_l_ankle_roll().set_goal(trim_to_range(outputs.left_ankle_roll* rad_to_degree));

        m_pose.get_r_hip_yaw().set_goal(trim_to_range(outputs.right_hip_yaw  * rad_to_degree));
        m_pose.get_r_hip_roll().set_goal(-trim_to_range(outputs.right_hip_roll * rad_to_degree));
        m_pose.get_r_hip_pitch().set_goal(trim_to_range(outputs.right_hip_pitch * rad_to_degree));
        m_pose.get_r_knee().set_goal(trim_to_range(outputs.right_knee * rad_to_degree));
        m_pose.get_r_ankle_pitch().set_goal(-trim_to_range(outputs.right_ankle_pitch  * rad_to_degree));
        m_pose.get_r_ankle_roll().set_goal(trim_to_range(outputs.right_ankle_roll * rad_to_degree));
    }
    if (stopRequest)
        set_velocity(0,0,0);
    if (phase < 1.5/engineFrequenzy)
    {
        if (stopRequest and std::abs(params.stepGain) < 0.001) {
            m_active = false;
            params.enabledGain = 0;
        }
        updateVelocity();
        updateGyro();
        std::cout << "Walk: Frequenz: " << engineFrequenzy << std::endl;
    }
    return  FootPhase::both;
}


