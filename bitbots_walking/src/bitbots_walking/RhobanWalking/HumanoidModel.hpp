#ifndef LEPH_HUMANOIDMODEL_HPP
#define LEPH_HUMANOIDMODEL_HPP

#include <Eigen/Dense>
#include "LegIK.hpp"

namespace Rhoban {

/**
 * Leg motor result positions
 */
struct IKWalkOutputs {
    double left_hip_yaw;
    double left_hip_roll;
    double left_hip_pitch;
    double left_knee;
    double left_ankle_pitch;
    double left_ankle_roll;
    double right_hip_yaw;
    double right_hip_roll;
    double right_hip_pitch;
    double right_knee;
    double right_ankle_pitch;
    double right_ankle_roll;
};

}

namespace Leph {

/**
 * All combinations
 * of Euler angles types
 * in same order as rotation application
 *
 * EulerYawPitchRoll is built as
 * Roll * Pitch * Yaw.
 */
enum EulerType {
    EulerYawPitchRoll,
    EulerYawRollPitch,
    EulerRollPitchYaw,
    EulerRollYawPitch,
    EulerPitchRollYaw,
    EulerPitchYawRoll,
};

/**
 * HumanoidModel
 */
class HumanoidModel
{
    public:

        /**
         * Initialize the model with given
         * typical leg length
         */
        HumanoidModel(
            double distHipToKnee,
            double distKneeToAnkle,
            double distAnkleToGround,
            double distFeetLateral);

        /**
         * Run analytical inverse kinematics LegIK and 
         * assign outputs motor positions.
         * Target footPos and angles are given in
         * special frame when all leg motors are in zero position.
         * Frame center is located on left or right foot tip.
         * True is returned if angles are updated and inverse
         * kinematics is sucessful, else false is returned.
         */
        bool legIkLeft(
            const Eigen::Vector3d& footPos, 
            const Eigen::Vector3d& angles,
            EulerType eulerType,
            Rhoban::IKWalkOutputs& outputs);
        bool legIkRight(
            const Eigen::Vector3d& footPos, 
            const Eigen::Vector3d& angles,
            EulerType eulerType,
            Rhoban::IKWalkOutputs& outputs);

        /**
         * Return the initial vertical distance
         * from trunk frame to foot tip frame (Z)
         */
        double legsLength() const;

        /**
         * Return the initial lateral distance
         * between each feet
         */
        double feetDistance() const;
    
    private:

        /**
         * Leg segments lengths used by
         * inverse kinematics
         */
        double _legHipToKnee;
        double _legKneeToAnkle;
        double _legAnkleToGround;
        double _legLateral;

        /**
         * Convert given euler angle of given
         * convention type to rotation matrix
         */
        Eigen::Matrix3d eulersToMatrix(
            const Eigen::Vector3d angles, EulerType eulerType) const;

        /**
         * Compute and return the IK position reference
         * vector and orientation reference matrix
         * in LegIK specifics structures
         */
        LegIK::Vector3D buildTargetPos(
            const Eigen::Vector3d& footPos);
        LegIK::Frame3D buildTargetOrientation(
            const Eigen::Vector3d& angles, 
            EulerType eulerType);

        /**
         * Assign model leg DOF to given IK results
         */
        void setIKResult(
            const LegIK::Position& result, bool isLeftLeg,
            Rhoban::IKWalkOutputs& outputs);

        /**
         * Check inverse kinematics computed value
         * and throw an error in case of NaN
         */
        void checkNaN(
            const LegIK::Position& result, 
            const LegIK::Vector3D& pos,
            const LegIK::Frame3D& orientation) const;
};

}

#endif

