#ifndef _ZMP_WALK_PARAMETER_H_
#define _ZMP_WALK_PARAMETER_H_

#include <Eigen/Core>

#include <boost/date_time/posix_time/posix_time.hpp>


namespace ZMPWalking{

struct ZMPParameter{
// Walk Parameters
// Stance and velocity limit values
Eigen::Vector2d stanceLimitX;
Eigen::Vector2d stanceLimitY;
Eigen::Vector2d stanceLimitA;
Eigen::Vector3d velDelta;


double velXHigh;
double velDeltaXHigh;

//Toe/heel overlap checking values
Eigen::Vector2d footSizeX;
double stanceLimitMarginY;

//Stance parameters
double bodyHeight;
double bodyTilt;
double footX;
double footY;
double supportX;
double supportY;
Eigen::Vector3d qLArm;
Eigen::Vector3d qRArm;

//Hardness parameters
double hardnessSupport;
double hardnessSwing;

double hardnessArm;
int pDefault;

//Gait parameters
double tStep;
double tZmp;
double stepHeight;
Eigen::Vector2d phSingle;

//Compensation parameters
double hipRollCompensation;
Eigen::Vector2d ankleMod;
double turnCompThreshold;
double turnComp;

//Gyro stabilization parameters
Eigen::Vector4d ankleImuParamX;
Eigen::Vector4d kneeImuParamX;
Eigen::Vector4d hipImuParamY;
Eigen::Vector4d armImuParamX;
Eigen::Vector4d armImuParamY;

//Support bias parameters to reduce backlash-based instability
double velFastForward;
double velFastTurn;
double supportFront;
double supportFront2;
double supportBack;
double supportSideX;
double supportSideY;
double supportTurn;
double ankleSupportFaktor;
double standOffset;

double frontComp;
double AccelComp;

double default_belly_pitch, default_belly_roll;

//Initial body swing
double supportModYInitial;

Eigen::Vector2d zPhase;
Eigen::Vector2d xPhase;
double bodyTilt_x_scaling;

EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ZMPParameter();

};

}//namespasce

ZMPWalking::ZMPParameter get_default_parameter();

#endif
