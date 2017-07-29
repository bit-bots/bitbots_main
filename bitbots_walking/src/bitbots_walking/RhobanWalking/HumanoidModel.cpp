#include "HumanoidModel.hpp"

namespace Leph {

HumanoidModel::HumanoidModel(
    double distHipToKnee,
    double distKneeToAnkle,
    double distAnkleToGround,
    double distFeetLateral)
{
    _legHipToKnee = distHipToKnee;
    _legKneeToAnkle = distKneeToAnkle;
    _legAnkleToGround = distAnkleToGround;
    _legLateral = distFeetLateral;
}
        
bool HumanoidModel::legIkLeft(
    const Eigen::Vector3d& footPos, 
    const Eigen::Vector3d& angles,
    EulerType eulerType,
    Rhoban::IKWalkOutputs& outputs)
{
    //LegIK initialization
    LegIK::IK ik(_legHipToKnee, 
        _legKneeToAnkle, _legAnkleToGround);
    //Convert foot position from given 
    //target to LegIK base
    LegIK::Vector3D legIKTarget = buildTargetPos(footPos);
    //Convert orientation from given frame
    //to LegIK base
    LegIK::Frame3D legIKMatrix = buildTargetOrientation(
        angles, eulerType);
    
    //Run inverse kinematics
    LegIK::Position result;
    bool isSucess = ik.compute(
        legIKTarget, legIKMatrix, result);

    //Update degrees of freedom on success
    if (isSucess) {
        checkNaN(result, legIKTarget, legIKMatrix);
        setIKResult(result, true, outputs);
    } 

    return isSucess;
}
bool HumanoidModel::legIkRight(
    const Eigen::Vector3d& footPos, 
    const Eigen::Vector3d& angles,
    EulerType eulerType,
    Rhoban::IKWalkOutputs& outputs)
{
    //LegIK initialization
    LegIK::IK ik(_legHipToKnee, 
        _legKneeToAnkle, _legAnkleToGround);
    //Convert foot position from given 
    //target to LegIK base
    LegIK::Vector3D legIKTarget = buildTargetPos(footPos);
    //Convert orientation from given frame
    //to LegIK base
    LegIK::Frame3D legIKMatrix = buildTargetOrientation(
        angles, eulerType);
    
    //Run inverse kinematics
    LegIK::Position result;
    bool isSucess = ik.compute(
        legIKTarget, legIKMatrix, result);

    //Update degrees of freedom on success
    if (isSucess) {
        checkNaN(result, legIKTarget, legIKMatrix);
        setIKResult(result, false, outputs);
    } 

    return isSucess;
}
        
double HumanoidModel::legsLength() const
{
    return _legHipToKnee + _legKneeToAnkle + _legAnkleToGround;
}
        
double HumanoidModel::feetDistance() const
{
    return _legLateral;
}

Eigen::Matrix3d HumanoidModel::eulersToMatrix(
    const Eigen::Vector3d angles, EulerType eulerType) const
{
    Eigen::Quaternion<double> quat;
    switch (eulerType) {
        case EulerYawPitchRoll: {
            Eigen::AngleAxisd yawRot(angles(0), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(1), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(2), Eigen::Vector3d::UnitX());
            quat = rollRot * pitchRot * yawRot;
        }
        break;
        case EulerYawRollPitch: {
            Eigen::AngleAxisd yawRot(angles(0), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(2), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(1), Eigen::Vector3d::UnitX());
            quat = pitchRot * rollRot * yawRot;
        }
        break;
        case EulerRollPitchYaw: {
            Eigen::AngleAxisd yawRot(angles(2), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(1), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(0), Eigen::Vector3d::UnitX());
            quat = yawRot * pitchRot * rollRot;
        }
        break;
        case EulerRollYawPitch: {
            Eigen::AngleAxisd yawRot(angles(1), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(2), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(0), Eigen::Vector3d::UnitX());
            quat = pitchRot * yawRot * rollRot;
        }
        break;
        case EulerPitchRollYaw: {
            Eigen::AngleAxisd yawRot(angles(2), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(0), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(1), Eigen::Vector3d::UnitX());
            quat = yawRot * rollRot * pitchRot;
        }
        break;
        case EulerPitchYawRoll: {
            Eigen::AngleAxisd yawRot(angles(1), Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchRot(angles(0), Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rollRot(angles(2), Eigen::Vector3d::UnitX());
            quat = rollRot * yawRot * pitchRot;
        }
        break;
    }
    return quat.matrix();
}

LegIK::Vector3D HumanoidModel::buildTargetPos(
    const Eigen::Vector3d& footPos)
{
    Eigen::Vector3d target;
    //Special frame where foot tip in zero position
    target = footPos;
    target.z() -= legsLength();

    //Building LegIK input target position 
    //data structure
    LegIK::Vector3D legIKTarget;
    legIKTarget[0] = target(0);
    legIKTarget[1] = target(1);
    legIKTarget[2] = target(2);
    return legIKTarget;
}
LegIK::Frame3D HumanoidModel::buildTargetOrientation(
    const Eigen::Vector3d& angles, 
    EulerType eulerType)
{
    Eigen::Matrix3d rotMatrixFrame = eulersToMatrix(
        angles, eulerType);
    Eigen::Matrix3d rotMatrixTarget = rotMatrixFrame;
    //Special frame where foot tip in zero position
    //No conversion

    //Building LegIK input target
    //orientation data structure
    LegIK::Frame3D legIKMatrix;
    legIKMatrix[0][0] = rotMatrixTarget(0, 0);
    legIKMatrix[0][1] = rotMatrixTarget(0, 1);
    legIKMatrix[0][2] = rotMatrixTarget(0, 2);
    legIKMatrix[1][0] = rotMatrixTarget(1, 0);
    legIKMatrix[1][1] = rotMatrixTarget(1, 1);
    legIKMatrix[1][2] = rotMatrixTarget(1, 2);
    legIKMatrix[2][0] = rotMatrixTarget(2, 0);
    legIKMatrix[2][1] = rotMatrixTarget(2, 1);
    legIKMatrix[2][2] = rotMatrixTarget(2, 2);
    return legIKMatrix;
}
        
void HumanoidModel::setIKResult(
    const LegIK::Position& result, bool isLeftLeg,
    Rhoban::IKWalkOutputs& outputs)
{
    if (isLeftLeg) {
        outputs.left_hip_yaw = result.theta[0];
        outputs.left_hip_roll = result.theta[1];
        outputs.left_hip_pitch = -result.theta[2];
        outputs.left_knee = result.theta[3];
        outputs.left_ankle_pitch = -result.theta[4];
        outputs.left_ankle_roll = result.theta[5];
    } else {
        outputs.right_hip_yaw = result.theta[0];
        outputs.right_hip_roll = result.theta[1];
        outputs.right_hip_pitch = -result.theta[2];
        outputs.right_knee = result.theta[3];
        outputs.right_ankle_pitch = -result.theta[4];
        outputs.right_ankle_roll = result.theta[5];
    }
}

void HumanoidModel::checkNaN(
    const LegIK::Position& result, 
    const LegIK::Vector3D& pos,
    const LegIK::Frame3D& orientation) const
{
    //Check if Nan is returned
    if (
        std::isnan(result.theta[0]) ||
        std::isnan(result.theta[1]) ||
        std::isnan(result.theta[2]) ||
        std::isnan(result.theta[3]) ||
        std::isnan(result.theta[4]) ||
        std::isnan(result.theta[5])
    ) {
        throw std::logic_error("LegIK NaN invalid result. "
            + std::string("theta0=") 
            + std::to_string(result.theta[0]) 
            + std::string(" ")
            + std::string("theta1=") 
            + std::to_string(result.theta[1]) 
            + std::string(" ")
            + std::string("theta2=") 
            + std::to_string(result.theta[2]) 
            + std::string(" ")
            + std::string("theta3=") 
            + std::to_string(result.theta[3]) 
            + std::string(" ")
            + std::string("theta4=") 
            + std::to_string(result.theta[4]) 
            + std::string(" ")
            + std::string("theta5=") 
            + std::to_string(result.theta[5]) 
            + std::string(" pos=")
            + pos.pp()
            + std::string(" orientation=")
            + orientation.pp()
        );
    }
}

}

