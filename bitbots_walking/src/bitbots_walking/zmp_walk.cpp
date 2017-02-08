#include <string>
#include <iostream>
#include <cmath>
#include <Eigen/Core>

#include "zmp_walk.hpp"
#include "zmp_walk_parameter.hpp"
#include "body.hpp"
#include "zmp_team_darwin_kinematics.hpp"
#include "zmp_math_basics.h"


using namespace boost::posix_time;
using namespace ZMPWalking;
using namespace Eigen;

using message::motion::kinematics::KinematicsModel;
using utility::motion::kinematics::calculateLegJointsTeamDarwin;

#include <assert.h>
#include "body.hpp"

#include "NUBots/utility/math/matrix/Transform3D.h"

inline double time_to_double(const boost::posix_time::time_duration& t) {
    long l = t.total_nanoseconds();
    assert(((double)l )/ e9 == ((double)l )/ e9);
    assert(((double)l )/ e9 != 0);
    return ((double)l )/ e9;
}



ZMPWalkWrapper* ZMPWalking::getInstance(const ZMPParameter& parameter)
{
return new ZMPWalk(parameter);
}


ZMPWalk::ZMPWalk(const ZMPParameter& parameter)
:kinematicsModel(), m_body(parameter.pDefault), m_parameter(parameter)
{

    m_uSupport = m_uTorso1 = m_uTorso2 =m_uTorso = Vector3d(parameter.supportX/2.0, 0, 0);
    m_uLeft1 = m_uLeft2 = m_uLeft = Vector3d(0, (parameter.footY), 0);
    m_uRight1 = m_uRight2 = m_uRight = Vector3d(0, -(parameter.footY), 0);


    m_velCurrent = Vector3d(0, 0, 0);
    m_velCommand = Vector3d(0, 0, 0);
    m_velDiff = Vector3d(0, 0, 0);

    //Gyro stabilization variables
    m_ankleShift = Vector2d(0, 0);
    m_kneeShift = 0;
    m_hipShift = Vector2d(0,0);
    m_armShift = Vector2d(0, 0);

    m_active = false;
    m_started = false;
    m_newStep = true;
    m_tLastStep = boost::posix_time::microsec_clock::local_time();

    m_stopRequest = 0;

    m_initial_step = 2;

    m_toeTipCompensation = 0;

    // Init Gyro to zero
    m_imuGyr = Vector3d::Zero();

    //ROS_DEBUG("Motion: Walk entry");
    //SJ: now we always assume that we start walking with feet together
    //Because joint readings are not always available with darwins
    stance_reset();

    //Place arms in appropriate position at sides
    m_body.set_larm_command(m_parameter.qLArm);
    m_body.set_rarm_command(m_parameter.qRArm);
    m_body.set_larm_hardness(m_parameter.hardnessArm);
    m_body.set_rarm_hardness(m_parameter.hardnessArm);


    m_velCurrent = Vector3d(0,0,0);
    m_velCommand = Vector3d(0,0,0);
}


ZMPWalk::FootPhase ZMPWalk::update() {
    if(not m_started) {
        m_started = true;
        m_tLastStep = boost::posix_time::microsec_clock::local_time();
    }
    boost::posix_time::ptime t = boost::posix_time::microsec_clock::local_time();

    double ph = time_to_double((t-m_tLastStep))/m_parameter.tStep;
    double phSingle = fmin(fmax(ph-m_parameter.zPhase.x(), 0)/(m_parameter.zPhase.y()-m_parameter.zPhase.x()),1);
    if(not m_active) { //Kommt aktuell nicht vor
        //ROS_DEBUG("Walking not active");
        return FootPhase::both;
    }

    //SJ: Variable tStep support for walkkick
    if(ph>=1) {
        ph = 0;
        m_newStep = true;
        m_tLastStep = boost::posix_time::microsec_clock::local_time();
    }


    //Stop when stopping sequence is done
    if(m_newStep and(m_stopRequest ==2)) {
        m_stopRequest = 0;
        m_active = false;
        //ROS_DEBUG("Walking is stopping");
        return FootPhase::both;
    }

    // New step
    if(m_newStep) {
        calculateStepGoal();
    } //end new step

    advanceInStep(ph,phSingle);

    return m_supportLeg ? FootPhase::right : FootPhase::left;
    // end motion_body
}

void ZMPWalk::calculateStepGoal()
{
    update_velocity();
    m_newStep = false;
    m_supportLeg = (m_supportLeg?0:1); // 0 for left support, 1 for right support
    m_uLeft1 = m_uLeft2;
    m_uRight1 = m_uRight2;
    m_uTorso1 = m_uTorso2;

    Vector2d supportMod = Vector2d(0,0); //Support Point modulation
    m_shiftFactor = 0.5; //How much should we shift final Torso pose?

    if(m_stopRequest ==1) {
        m_stopRequest = 2;
        m_velCurrent = Vector3d(0,0,0);
        m_velCommand = Vector3d (0,0,0);
        if(m_supportLeg == 0) {        // Left support
            m_uRight2 = pose_global(-2*Vector3d(m_parameter.standOffset,m_parameter.footY,0), m_uLeft1); //TODO Config
        } else {       // Right support
            m_uLeft2 = pose_global(2*Vector3d(m_parameter.standOffset,m_parameter.footY,0), m_uRight1);
        }
    } else { //Normal walk, advance steps
        if(m_supportLeg == 0) { // Left support
            m_uRight2 = step_right_destination(m_velCurrent, m_uLeft1, m_uRight1);
        } else { // Right support
            m_uLeft2 = step_left_destination(m_velCurrent, m_uLeft1, m_uRight1);
        }
        //Velocity-based support point modulation
        m_toeTipCompensation = 0;
        if((m_velDiff.x()>0) && (m_velCurrent.x() > 0)) { //Accelerating to front
            supportMod.x() = m_parameter.supportFront2;
        } else if(m_velCurrent.x()>m_parameter.velFastForward) {
            supportMod.x() = m_parameter.supportFront;
            m_toeTipCompensation = m_ankleMod.x();
        } else if((m_velCurrent.x()<0)) {
            supportMod.x() = m_parameter.supportBack;
        } else if(std::abs(m_velCurrent.z())>m_parameter.velFastTurn) {
            supportMod.x() = m_parameter.supportTurn;
        } else {
            if(m_velCurrent.y()>0.015) {
                supportMod.x() = m_parameter.supportSideX;
                supportMod.y() = m_parameter.supportSideY;
            } else if(m_velCurrent.y()<-0.015) {
                supportMod.x() = m_parameter.supportSideX;
                supportMod.y() = -m_parameter.supportSideY;
            }
        }
    }

    m_uTorso2 = step_torso(m_uLeft2, m_uRight2,m_shiftFactor);

    //Adjustable initial step body swing
    if(m_initial_step>0) {
        //ROS_DEBUG(" init steps") ;
        if(m_supportLeg == 0) { //LS
            supportMod.y() = m_parameter.supportModYInitial;
        } else { //RS
            supportMod.y() = -m_parameter.supportModYInitial;
        }
    }
    Eigen::Vector3d supportOffset = m_supportLeg?m_uRight1:m_uLeft1;

    if (m_robottype == "Darwin") {
    supportOffset.y() +=  (m_supportLeg?+1:-1)* m_parameter.footY;
    }
    //Apply velocity-based support point modulation for uSupport
    Vector3d uFootTorso = pose_relative(supportOffset,m_uTorso1); //Weg vom noch hinteren Beim zum AnfangsSchwerpunkt
    Vector3d uTorsoModded = pose_global(Vector3d (supportMod.x(),supportMod.y(),0),m_uTorso);
    Vector3d uFootModded = pose_global (uFootTorso,uTorsoModded);
    m_uSupport = pose_global(Vector3d (m_parameter.supportX, (m_supportLeg?-1:1)*m_parameter.supportY, 0),uFootModded);
    //m_body.set_lleg_hardness(m_supportLeg?m_parameter.hardnessSwing:m_parameter.hardnessSupport);
    //m_body.set_rleg_hardness(m_supportLeg?m_parameter.hardnessSupport:m_parameter.hardnessSwing);

    //Compute ZMP coefficients
    m_ZMP_coeff.m1X = (m_uSupport.x()-m_uTorso1.x())/(m_parameter.tStep*m_parameter.phSingle.x()); //Eigentlich Linearen Trajektori Wert
    m_ZMP_coeff.m2X = (m_uTorso2.x()-m_uSupport.x())/(m_parameter.tStep*(1-m_parameter.phSingle.y()));
    m_ZMP_coeff.m1Y = (m_uSupport.y()-m_uTorso1.y())/(m_parameter.tStep*m_parameter.phSingle.x());
    m_ZMP_coeff.m2Y = (m_uTorso2.y()-m_uSupport.y())/(m_parameter.tStep*(1-m_parameter.phSingle.y()));
    Vector2d zmp_solve_result = zmp_solve(m_uSupport.x(), m_uTorso1.x(), m_uTorso2.x(), m_uTorso1.x(), m_uTorso2.x());
    m_ZMP_coeff.aXP = zmp_solve_result.x();
    m_ZMP_coeff.aXN = zmp_solve_result.y();
    zmp_solve_result = zmp_solve(m_uSupport.y(), m_uTorso1.y(), m_uTorso2.y(), m_uTorso1.y(), m_uTorso2.y());

    m_ZMP_coeff.aYP = zmp_solve_result.x();
    m_ZMP_coeff.aYN = zmp_solve_result.y();


}

void ZMPWalk::advanceInStep(double ph, double phSingle){
      Vector2d xzFoot = foot_phase(ph,phSingle);
      double xFoot = xzFoot.x();
      double zFoot = xzFoot.y();

      if(m_initial_step>0) { zFoot = 0; } //Don't lift foot at initial step

      if(m_supportLeg == 0) {    // Left support
          m_uRight = se2_interpolate(xFoot, m_uRight1, m_uRight2);
      } else {    // Right support
          m_uLeft = se2_interpolate(xFoot, m_uLeft1, m_uLeft2);
      }
      m_uTorso = zmp_com(ph, m_uSupport, m_ZMP_coeff);

      //Turning
      double turnCompX = 0;
      if (std::abs(m_velCurrent.z())>m_parameter.turnCompThreshold and m_velCurrent.x()>-0.01) {
          turnCompX = m_parameter.turnComp;
      }

      //Walking front
      double frontCompX = 0;
      if((m_velCurrent.x()>0.04)) {
          frontCompX = m_parameter.frontComp;
      }
      if((m_velDiff.x()>0.02)) {
          frontCompX = frontCompX + m_parameter.AccelComp;
      }


      m_uTorsoActual = pose_global(Vector3d (m_parameter.footX+frontCompX+turnCompX,0,0), m_uTorso);
      Eigen::Matrix<double, 6, 1> pLLeg, pRLeg, pTorso;


      pTorso = Matrix<double, 6, 1>();
      pTorso.head<2>() = m_uTorsoActual.head<2>();
      pTorso(2) = m_parameter.bodyHeight;
      pTorso.tail<3>() = Vector3d(0, m_bodyTilt , m_uTorsoActual.z());
      //Arm movement compensation
      if (m_robottype == "Hambot")
          pTorso(4) = m_bodyTilt * (sin(ph*2*pi) + 1) ;


      pLLeg = Matrix<double, 6, 1>();
      pLLeg.head<2>() = m_uLeft.head<2>();
      pLLeg(2) = 0; pLLeg(3) = 0; pLLeg(4) = 0;
      pLLeg(5) = m_uLeft.z();


      pRLeg = Matrix<double, 6, 1>();
      pRLeg.head<2>() = m_uRight.head<2>();
      pRLeg(2) = 0; pRLeg(3) = 0; pRLeg(4) = 0;
      pRLeg(5) = m_uRight.z();

      //Transform2D uTorsoActual = uTorso.localToWorld({-kinematicsModel.Leg.HIP_OFFSET_X, 0, 0});
      utility::math::matrix::Transform3D torso = arma::vec6({m_uTorsoActual.x(), m_uTorsoActual.y(), m_parameter.bodyHeight, 0, m_bodyTilt, m_uTorsoActual.z()});

      std::vector<double> v_l = {m_uLeft.x(), m_uLeft.y()};
      utility::math::matrix::Transform2D leftFoot_2D{arma::vec(v_l),m_uLeft.z()};
      utility::math::matrix::Transform3D leftFoot{leftFoot_2D};

      std::vector<double> v_r = {m_uRight.x(), m_uRight.y()};
      utility::math::matrix::Transform2D rightFoot_2D{arma::vec(v_r),m_uRight.z()};
      utility::math::matrix::Transform3D rightFoot{rightFoot_2D};

      if(m_supportLeg == 0) {    // Left support
          pRLeg(2) = m_parameter.stepHeight*zFoot;
          rightFoot = rightFoot.translateZ(m_parameter.stepHeight*zFoot);
      } else {    // Right support
          pLLeg(2) = m_parameter.stepHeight*zFoot;
          leftFoot = leftFoot.translateZ(m_parameter.stepHeight*zFoot);
      }


      utility::math::matrix::Transform3D leftFootTorso = leftFoot.worldToLocal(torso);
      utility::math::matrix::Transform3D rightFootTorso = rightFoot.worldToLocal(torso);

      std::vector<std::pair<message::input::ServoID, float>> joints = calculateLegJointsTeamDarwin(kinematicsModel, leftFootTorso, rightFootTorso);
       Matrix<double, 12, 1> qLegs;
       if (true)
         {
       qLegs  = Matrix<double, 12, 1>::Zero();
      for (std::pair<message::input::ServoID, float> &motor: joints)
      {
        switch(motor.first)
        {
        case 6:
        qLegs(6) = motor.second;
        break;
        case 7:
        qLegs(0) = motor.second;
        break;
        case 8:
        qLegs(7) = motor.second;
        break;
        case 9:
        qLegs(1) = motor.second;
        break;
        case 10:
        qLegs(8) = motor.second;
        break;
        case 11:
        qLegs(2) = motor.second;
        break;
        case 12:
        qLegs(9) = motor.second;
        break;
        case 13:
        qLegs(3) = motor.second;
        break;
        case 14:
        qLegs(10) = motor.second;
        break;
        case 15:
        qLegs(4) = motor.second;
        break;
        case 16:
        qLegs(11) = motor.second;
        break;
        case 17:
        qLegs(5) = motor.second;
        break;
         }
      }
         }
         else
         {
          qLegs = inverse_legs(pLLeg, pRLeg, pTorso);
         }
      motion_legs(qLegs, phSingle);

      motion_arms();
}

void ZMPWalk::applyGyroStabilization(Eigen::Matrix<double, 12, 1>& qLegs) {
    //Ankle stabilization using gyro feedback

    double gyro_roll0 = m_imuGyr.x(); //0 if without Gyrostabilisation
    double gyro_pitch0 = m_imuGyr.y();

    //get effective gyro angle considering body angle offset
    double yawAngle = 0;
    if (not m_active) { //double support
        yawAngle = (m_uLeft.z() + m_uRight.z()) / 2.0 - m_uTorsoActual.z();
    } else if (m_supportLeg == 0) {  // Left support
        yawAngle = m_uLeft.z() - m_uTorsoActual.z();
    } else if (m_supportLeg == 1) {
        yawAngle = m_uRight.z() - m_uTorsoActual.z();
    }
    double gyro_roll = gyro_roll0 * cos(yawAngle) - gyro_pitch0 * sin(yawAngle);
    double gyro_pitch = gyro_pitch0 * cos(yawAngle) - gyro_roll0 * sin(yawAngle);
    double armShiftX = procFunc(gyro_pitch * m_parameter.armImuParamX(1), m_parameter.armImuParamX(2),
                                m_parameter.armImuParamX(3));
    double armShiftY = procFunc(gyro_roll * m_parameter.armImuParamY(1), m_parameter.armImuParamY(2),
                                m_parameter.armImuParamY(3));

    double ankleShiftX = procFunc(gyro_pitch * m_parameter.ankleImuParamX(1), m_parameter.ankleImuParamX(2),
                                  m_parameter.ankleImuParamX(3));
    double ankleShiftY = procFunc(gyro_roll * m_parameter.ankleImuParamX(1), m_parameter.ankleImuParamX(2),
                                  m_parameter.ankleImuParamX(3));
    double kneeShiftX = procFunc(gyro_pitch * m_parameter.kneeImuParamX(1), m_parameter.kneeImuParamX(2),
                                 m_parameter.kneeImuParamX(3));
    double hipShiftY = procFunc(gyro_roll * m_parameter.hipImuParamY(1), m_parameter.hipImuParamY(2),
                                m_parameter.hipImuParamY(3));

    m_ankleShift.x() = m_ankleShift.x() + m_parameter.ankleImuParamX(0) * (ankleShiftX - m_ankleShift.x());
    m_ankleShift.y() = m_ankleShift.y() + m_parameter.ankleImuParamX(0) * (ankleShiftY - m_ankleShift.y());
    m_kneeShift = m_kneeShift + m_parameter.kneeImuParamX(0) * (kneeShiftX - m_kneeShift);
    m_hipShift.y() = m_hipShift.y() + m_parameter.hipImuParamY(0) * (hipShiftY - m_hipShift.y());
    m_armShift.x() = m_armShift.x() + m_parameter.armImuParamX(0) * (armShiftX - m_armShift.x());
    m_armShift.y() = m_armShift.y() + m_parameter.armImuParamY(0) * (armShiftY - m_armShift.y());


    //TODO: Toe/heel lifting

    if (not m_active) { //Double support, standing still
        //qLegs(2) = qLegs(2) + hipShift.y();    //Hip roll stabilization
        qLegs(Body::LKNEE) += m_kneeShift;    //Knee pitch stabilization
        qLegs(Body::LANKLE_PITCH) += m_ankleShift.x();    //Ankle pitch stabilization
        //qLegs(5) = qLegs(5) + ankleShift.y();    //Ankle roll stabilization

        //qLegs(7) = qLegs(7)  + hipShift.y();    //Hip roll stabilization
        qLegs(Body::RKNEE) += m_kneeShift;    //Knee pitch stabilization
        qLegs(Body::RANKLE_PITCH) += m_ankleShift.x();    //Ankle pitch stabilization
        //qLegs(11) = qLegs(11) + ankleShift.y();    //Ankle roll stabilization
    } else {
        if (m_supportLeg == 0) {  // Left support
            qLegs(Body::RHIP_ROLL) += m_hipShift.y();    //Hip roll stabilization
            qLegs(Body::LKNEE) += m_kneeShift;    //Knee pitch stabilization
            qLegs(Body::LANKLE_PITCH) += m_ankleShift.x();    //Ankle pitch stabilization
            qLegs(Body::LANKLE_ROLL) = (qLegs(Body::LANKLE_ROLL) + m_ankleShift.y()) *
                                       m_parameter.ankleSupportFaktor;    //Ankle roll stabilization
            qLegs(Body::RANKLE_ROLL) *= m_parameter.ankleSupportFaktor;
        } else {
            qLegs(Body::RHIP_ROLL) -= m_hipShift.y();    //Hip roll stabilization
            qLegs(Body::RKNEE) += m_kneeShift;    //Knee pitch stabilization
            qLegs(Body::RANKLE_PITCH) += m_ankleShift.x();    //Ankle pitch stabilization
            qLegs(Body::RANKLE_ROLL) = (qLegs(Body::RANKLE_ROLL) + m_ankleShift.y()) *
                                       m_parameter.ankleSupportFaktor;    //Ankle roll stabilization
            qLegs(Body::LANKLE_ROLL) *= m_parameter.ankleSupportFaktor;
        }
    }
}

void ZMPWalk::motion_legs(Matrix<double, 12, 1> &qLegs, double phSingle) {
    double phComp = fmin(1.0, fmin(phSingle / .1, (1 - phSingle) / .1));
    assert(phComp == phComp);
    applyGyroStabilization(qLegs);


    double belly_roll = m_parameter.default_belly_roll;
    double belly_pitch = m_parameter.default_belly_pitch;

    if (m_active) {
        if (m_supportLeg == 0) {
            qLegs(Body::RANKLE_PITCH) += m_toeTipCompensation * phComp;//Lifting toetip
            qLegs(Body::LHIP_ROLL) += m_parameter.hipRollCompensation * phComp; //Hip roll compensation
        } else {
            qLegs(Body::LANKLE_PITCH) += m_toeTipCompensation * phComp;//Lifting toetip
            qLegs(Body::RHIP_ROLL) -= m_parameter.hipRollCompensation * phComp;//Hip roll compensation TODO geändert
        }
    }


    if (m_robottype == "Hambot") {
        double diff_roll = qLegs(Body::RHIP_ROLL) - qLegs(Body::LHIP_ROLL);
        belly_roll = -1 * (qLegs(Body::RHIP_ROLL) + qLegs(Body::LHIP_ROLL)) / 2;
        qLegs(Body::RHIP_ROLL) = diff_roll / 2;
        qLegs(Body::LHIP_ROLL) = -diff_roll / 2;

        double diff_pitch = qLegs(Body::RHIP_PITCH) - qLegs(Body::LHIP_PITCH);
        belly_pitch = -1 * (qLegs(Body::RHIP_PITCH) + qLegs(Body::LHIP_PITCH)) / 2;
        qLegs(Body::RHIP_PITCH) = diff_pitch / 2;
        qLegs(Body::LHIP_PITCH) = -diff_pitch / 2;
    }

    m_body.set_lleg_command(qLegs);
    m_body.set_belly_to_initial(belly_pitch, belly_roll);
}

void ZMPWalk::motion_arms() {

    Vector3d qLArmActual;
    Vector3d qRArmActual;

    qLArmActual.head<2>() = m_parameter.qLArm.head<2>()+m_armShift.head<2>();
    qRArmActual.head<2>() = m_parameter.qRArm.head<2>()+m_armShift.head<2>();

    //Check leg hitting
    double RotLeftA =  mod_angle(m_uLeft.z() - m_uTorso.z());
    double RotRightA =  mod_angle(m_uTorso.z() - m_uRight.z());

    Vector3d LLegTorso = pose_relative(m_uLeft,m_uTorso);
    Vector3d RLegTorso = pose_relative(m_uRight,m_uTorso);

    qLArmActual.y() = fmax(5* degree_to_rad + fmax(0, RotLeftA)/2
        + fmax(0,LLegTorso.y() - 0.04) /0.02 * 6* degree_to_rad,qLArmActual.y());
    qRArmActual.y() = fmin(-5* degree_to_rad - fmax(0, RotRightA)/2
        - fmax(0,-RLegTorso.y() - 0.04)/0.02 * 6* degree_to_rad, qRArmActual.y());

    qLArmActual.z() = m_parameter.qLArm.z();
    qRArmActual.z() = m_parameter.qRArm.z();
    m_body.set_larm_command((Vector3d)qLArmActual);
    m_body.set_rarm_command((Vector3d)qRArmActual);
    return;
}

Vector3d ZMPWalk::step_left_destination(const Vector3d& vel, const Vector3d& uLeft, const Vector3d& uRight) {
    Vector3d u0 = se2_interpolate(.5, uLeft, uRight);
    // Determine nominal midpoint position 1.5 steps in future
    Vector3d u1 = pose_global(1.5*vel, u0);
    Vector3d uLeftPredict = pose_global(Vector3d(0,m_parameter.footY,0), u1);
    Vector3d uLeftRight = pose_relative(uLeftPredict, uRight);
    // Do not pidgeon toe, cross feet:

    //Check toe and heel overlap
    double toeOverlap = -m_parameter.footSizeX.x()*uLeftRight.z();
    double heelOverlap = -m_parameter.footSizeX.y()*uLeftRight.z();
    double limitY = fmax(m_parameter.stanceLimitY.x(),2* m_parameter.footY-m_parameter.stanceLimitMarginY+fmax(toeOverlap,heelOverlap));


    uLeftRight.x() = fmin(fmax(uLeftRight.x(), m_parameter.stanceLimitX.x()), m_parameter.stanceLimitX.y());
    uLeftRight.y() = fmin(fmax(uLeftRight.y(), limitY),m_parameter.stanceLimitY.y());
    uLeftRight.z() = fmin(fmax(uLeftRight.z(), m_parameter.stanceLimitA.x()), m_parameter.stanceLimitA.y());


    Vector3d tmp = pose_global(uLeftRight, uRight);
    return tmp;
}

Vector3d ZMPWalk::step_right_destination(const Vector3d& vel, const Vector3d& uLeft, const Vector3d& uRight) {
    Vector3d u0 = se2_interpolate(.5, uLeft, uRight);
    // Determine nominal midpoint position 1.5 steps in future
    Vector3d u1 = pose_global(1.5*vel, u0);
    Vector3d uRightPredict = pose_global(-1*Vector3d(0,m_parameter.footY,0), u1);
    Vector3d uRightLeft = pose_relative(uRightPredict, uLeft);
    // Do not pidgeon toe, cross feet:

    //Check toe and heel overlap
    double toeOverlap = m_parameter.footSizeX.x()*uRightLeft.z();
    double heelOverlap = m_parameter.footSizeX.y()*uRightLeft.z();
    double limitY = fmax(m_parameter.stanceLimitY.x(), 2* m_parameter.footY-m_parameter.stanceLimitMarginY+fmax(toeOverlap,heelOverlap));


    uRightLeft.x() = fmin(fmax(uRightLeft.x(), m_parameter.stanceLimitX.x()), m_parameter.stanceLimitX.y());
    uRightLeft.y() = fmin(fmax(uRightLeft.y(), -m_parameter.stanceLimitY.y()), -limitY);
    uRightLeft.z() = fmin(fmax(uRightLeft.z(), -m_parameter.stanceLimitA.y()), -m_parameter.stanceLimitA.x());

    Vector3d tmp = pose_global(uRightLeft, uLeft);
    return tmp;
}


Vector3d ZMPWalk::step_torso(const Vector3d& uLeft, const Vector3d& uRight, double shiftFactor) {
    Vector3d uLeftSupport = pose_global(Vector3d(m_parameter.supportX, m_parameter.supportY, 0), uLeft);
    Vector3d uRightSupport = pose_global(Vector3d(m_parameter.supportX, -m_parameter.supportY, 0), uRight);
    return se2_interpolate(shiftFactor, uLeftSupport, uRightSupport);
}


void ZMPWalk::set_velocity(double vx, double vy, double va) {
    //Filter the commanded speed
    m_velCommand.x() = vx;
    m_velCommand.y() = vy;
    m_velCommand.z() = va;
}


void ZMPWalk::update_velocity() {
    if(m_velCurrent.x()>m_parameter.velXHigh) {
        //Slower accelleration at high speed
        m_velDiff.x() = fmin(fmax(m_velCommand.x()-m_velCurrent.x(),-m_parameter.velDelta.x()),m_parameter.velDeltaXHigh);
    } else {
        m_velDiff.x() = fmin(fmax(m_velCommand.x()-m_velCurrent.x(), -m_parameter.velDelta.x()),m_parameter.velDelta.x());
    }
    m_velDiff.y() = fmin(fmax(m_velCommand.y()-m_velCurrent.y(), -m_parameter.velDelta.y()),m_parameter.velDelta.y());
    m_velDiff.z() = fmin(fmax(m_velCommand.z()-m_velCurrent.z(), -m_parameter.velDelta.z()),m_parameter.velDelta.z());

    m_velCurrent.x() = m_velCurrent.x()+m_velDiff.x();
    m_velCurrent.y() = m_velCurrent.y()+m_velDiff.y();
    m_velCurrent.z() = m_velCurrent.z()+m_velDiff.z();

    if(m_initial_step>0) {
        m_velCurrent = Vector3d (0,0,0);
        m_initial_step = m_initial_step-1;
    }
}

void ZMPWalk::start() {
    m_stopRequest = 0;
    if(not m_active) {
        m_active = true;
        m_started = false;
        m_newStep = true;
        m_tLastStep = boost::posix_time::microsec_clock::local_time();
        m_initial_step = 2;
    }
}


void ZMPWalk::stop() {
    //Always stops with feet together (which helps transition)
    m_stopRequest = fmax(1,m_stopRequest);
}

/* standup/sitdown/falldown handling */
void ZMPWalk::stance_reset() {

    //ROS_DEBUG(2, "Stance Resetted");


    m_uTorso1 = m_uTorso2 =m_uTorso = Vector3d(m_parameter.supportX/2.0, 0, 0);
    m_uLeft1 = m_uLeft2 = m_uLeft = Vector3d(0, (m_parameter.footY), 0);
    m_uRight1 = m_uRight2 = m_uRight = Vector3d(0, -(m_parameter.footY), 0);

    m_newStep = true;
    m_uSupport = m_uTorso;
    m_tLastStep = boost::posix_time::microsec_clock::local_time();
}

//m_uSupport.x(), m_uTorso1.x(), m_uTorso2.x(), m_uTorso1.x(), m_uTorso2.x()
Vector2d ZMPWalk::zmp_solve(double zs, double z1, double z2, double x1, double x2) {
    /*
    Solves ZMP equation:
    x(t) = z(t) + aP*exp(t/tZmp) + aN*exp(-t/tZmp) - tZmp*mi*sinh((t-Ti)/tZmp)
    where the ZMP point is piecewise linear:
    z(0) = z1, z(T1 < t < T2) = zs, z(tStep) = z2
    */
    double T1 = m_parameter.tStep*m_parameter.phSingle.x();
    double T2 = m_parameter.tStep*m_parameter.phSingle.y();
    double m1 = (zs-z1)/T1;
    double m2 = -(zs-z2)/(m_parameter.tStep-T2);

    double c1 = x1-z1+m_parameter.tZmp*m1*sinh(-T1/m_parameter.tZmp);
    double c2 = x2-z2+m_parameter.tZmp*m2*sinh((m_parameter.tStep-T2)/m_parameter.tZmp);
    double expTStep = exp(m_parameter.tStep/m_parameter.tZmp);
    double aP = (c2 - c1/expTStep)/(expTStep-1/expTStep);
    double aN = (c1*expTStep - c2)/(expTStep-1/expTStep);
    return Vector2d(aP, aN);
}

//Finds the necessary COM for stability and returns it
Vector3d ZMPWalk::zmp_com(double ph, const Eigen::Vector3d& uSupport, const ZMP_coeff& zmp_coeff) {

    Vector3d com = Vector3d(0, 0, 0);
    double expT = exp(m_parameter.tStep*ph/m_parameter.tZmp);
    //Debug Info: expT liegt bei m_tStep=0.6 und m_tZmp=0.45 zwischen 1 und 4.5
    com.x() = uSupport.x() + zmp_coeff.aXP*expT + zmp_coeff.aXN/expT;
    com.y() = uSupport.y() + zmp_coeff.aYP*expT + zmp_coeff.aYN/expT;
    if(ph < m_parameter.phSingle.x()) {
        com.x() = com.x() + zmp_coeff.m1X*m_parameter.tStep*(ph-m_parameter.phSingle.x())
            - m_parameter.tZmp*zmp_coeff.m1X*sinh(m_parameter.tStep*(ph-m_parameter.phSingle.x())/m_parameter.tZmp);
        com.y() = com.y() + zmp_coeff.m1Y*m_parameter.tStep*(ph-m_parameter.phSingle.x())
            -m_parameter.tZmp*zmp_coeff.m1Y*sinh(m_parameter.tStep*(ph-m_parameter.phSingle.x())/m_parameter.tZmp);
    } else if(ph > m_parameter.phSingle.y()) {
        com.x() = com.x() + zmp_coeff.m2X*m_parameter.tStep*(ph-m_parameter.phSingle.y())
            -m_parameter.tZmp*zmp_coeff.m2X*sinh(m_parameter.tStep*(ph-m_parameter.phSingle.y())/m_parameter.tZmp);
        com.y() = com.y() + zmp_coeff.m2Y*m_parameter.tStep*(ph-m_parameter.phSingle.y())
            -m_parameter.tZmp*zmp_coeff.m2Y*sinh(m_parameter.tStep*(ph-m_parameter.phSingle.y())/m_parameter.tZmp);
    }
    com.z() = ph* (m_uLeft2.z()+m_uRight2.z())*0.5 + (1-ph)* (m_uLeft1.z()+m_uRight1.z())*0.5;
    return com;
}

Vector2d ZMPWalk::foot_phase(double ph, double phSingle) {
    // Computes relative x,z motion of foot during single support phase
    // phSingle = 0: x = 0, z = 0, phSingle = 1: x = 1,z = 0
    double phSingleSkewZ = pow(phSingle, 0.8) - 0.17*phSingle*(1-phSingle);
    double zf = 0.5*(1-cos(2*pi*phSingleSkewZ));

    double phSingleX = fmin(fmax(ph-m_parameter.xPhase.x(), 0)/(m_parameter.xPhase.y()-m_parameter.xPhase.x()),1);
    double phSingleSkewX = pow(phSingleX, 0.8) - 0.17*phSingleX*(1-phSingleX);
    double xf = 0.5*(1-cos(pi*phSingleSkewX));

    return Vector2d(xf, zf);
}

/**
 * computes a mod, operation designed for angles in the range -pi, pi
 */
double ZMPWalk::mod_angle(double a) {
    //Reduce angle to [-pi, pi)
    a = fmod(a, (2*pi));
    if (a >= pi) {
        a = a - 2*pi;
    }
    return a;
}

/**
 * Computes the sum out of two so called poses in an absolute way
 * @param pRelative the relative pose that is added on the other
 * @param pose the pose that is used as basis
 * The "relative" pose is rotated around their own z-component and then
 * added to the global other pose. Then the two rotation parts are summed up.
 * //Note from @Robert: I'm a bit confused why the rotation components are summed up.
 *      It still can be, that my imagination of the internal pose representation is
 *      still false, but this method would implicitly perform any kind of
 *      rotation of the original pose, without performing the maths to keep the
 *      components aligned with each other.
 * @return The summed up pose of the two input poses
 */
Vector3d ZMPWalk::pose_global(const Vector3d& pRelative, const Vector3d& pose) {
    double ca = cos(pose.z());
    double sa = sin(pose.z());
    return Vector3d(pose.x() + ca*pRelative.x() - sa*pRelative.y(),
                    pose.y() + sa*pRelative.x() + ca*pRelative.y(),
                    pose.z() + pRelative.z());
}

/**
 * Computes a relative sum of two poses
 * @param pGlobal The global pose of the update as reference
 * @param pose The pose to work on
 * First the difference between the two position parts of the poses is performed.
 * The pose is subtracted from the global goal. Then this difference is
 * rotated by the poses angle. Finally the resulting angle is the difference of
 * the two poses rotations.
 * @return The summed up pose of the two input poses
 */
// pose "+" returnValue = pGlobal
// pose_global_new( a, b) = c
// pose_relative(c, a) = b
// Berechnet den relativen Vector von pose nach pGlobal inklusive entsprechendere Drehung
Vector3d ZMPWalk::pose_relative(const Vector3d& pGlobal, const Vector3d& pose) {
    double ca = cos(-pose.z());
    double sa = sin(-pose.z());
    double px = pGlobal.x()-pose.x();
    double py = pGlobal.y()-pose.y();
    double pa = pGlobal.z()-pose.z();
    return Vector3d(ca*px - sa*py, sa*px + ca*py, mod_angle(pa));
}

/**
 * This function looks like a deadband function, but behaves a little different.
 * The method reduces the norm of any input @param a by the parameter @param deadband
 * without switching signs. But the result will never be higher than @param maxvalue.
 * Of course this is meant in the means of the absolute value preserving the signs.
 */
double ZMPWalk::procFunc(double a, double deadband, double maxvalue) {
    //Piecewise linear function for IMU feedback
    double b;
    if (a>0) {
        b=fmin(fmax(0,std::abs(a)-deadband), maxvalue);
    } else {
        b=-fmin(fmax(0,std::abs(a)-deadband), maxvalue);
    }
    return b;
}

/**
 * @param t a scaling parameter how to interpolate between u1 and u2
 * @param u1 the basic parameter. This is the basic part of the result
 * @param u2 the part to give the maximum interpolation target.
 * @return an interpolation between u1 and u2. t=0 => r=u1, t<1 => u1<r<u2 t=1 =>r=u2
 */
Vector3d ZMPWalk::se2_interpolate(double t, const Vector3d& u1, const Vector3d& u2) {
      // helps smooth out the motions using a weighted average
      return Vector3d(u1.x()+t*(u2.x()-u1.x()),
                    u1.y()+t*(u2.y()-u1.y()),
                    u1.z()+t*mod_angle(u2.z()-u1.z()));
}



double ZMPWalk::get_hip_pitch(){return m_bodyTilt * rad_to_degree;}
Robot::Pose& ZMPWalk::get_pose(){return m_body.get_pose();}
double ZMPWalk::get_l_shoulder_pitch_offset(){return m_body.get_l_shoulder_pitch_offset();}
double ZMPWalk::get_r_shoulder_pitch_offset(){return m_body.get_r_shoulder_pitch_offset();}
double ZMPWalk::get_l_shoulder_roll_offset(){return m_body.get_l_shoulder_roll_offset();}
double ZMPWalk::get_r_shoulder_roll_offset(){return m_body.get_r_shoulder_roll_offset();}
double ZMPWalk::get_hip_pitch_offset(){return m_body.get_hip_pitch_offset();}
std::string ZMPWalk::get_robottype(){return m_robottype;}
const Eigen::Vector3d& ZMPWalk::get_uLeft(){return m_uLeft;}
const Eigen::Vector3d& ZMPWalk::get_uRight(){return m_uRight;}
const Eigen::Vector3d& ZMPWalk::get_velocity(){return m_velCurrent;}
bool ZMPWalk::is_active(){return m_active;};
void ZMPWalk::set_active(bool active){this->m_active = active;}
void ZMPWalk::set_hip_pitch(double hip_pitch){m_bodyTilt = hip_pitch * degree_to_rad;}
void ZMPWalk::set_l_shoulder_pitch_offset(double offset){m_body.set_l_shoulder_pitch_offset(offset);}
void ZMPWalk::set_r_shoulder_pitch_offset(double offset){m_body.set_r_shoulder_pitch_offset(offset);}
void ZMPWalk::set_l_shoulder_roll_offset(double offset){m_body.set_l_shoulder_roll_offset(offset);}
void ZMPWalk::set_r_shoulder_roll_offset(double offset){m_body.set_r_shoulder_roll_offset(offset);}
void ZMPWalk::set_hip_pitch_offset(double offset){m_body.set_hip_pitch_offset(offset);}
void ZMPWalk::set_gyro_data(const Eigen::Vector3d& gyro){m_imuGyr = gyro;}
void ZMPWalk::set_long_legs(double thigh, double tibia, double hip_y_offset, double hip_z_offset, double foot_height) {
set_long_leg_adjusted_values(thigh, tibia, hip_y_offset, hip_z_offset, foot_height);
}
void ZMPWalk::set_belly_roll_pid(float p, float i, float d)
{
m_body.set_belly_roll_pid(p,i,d);
}
void ZMPWalk::set_robottype(std::string robottype){m_robottype = robottype;}
