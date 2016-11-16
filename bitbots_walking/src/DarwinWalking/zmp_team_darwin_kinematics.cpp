#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <iostream>
#include "zmp_team_darwin_kinematics.hpp"
#include "kinematic_basics.hpp"

enum {LEG_LEFT = 0, LEG_RIGHT = 1};

static double hipOffsetY = .037;    //OP, measured
static double hipOffsetZ = .096;    //OP, Calculated from spec
static double thighLength = .0930;  //OP, spec //Für angepassten Darwin überschreibbar machen
static double tibiaLength = .0930;  //OP, spec //Für angepassten Darwin überschreibbar machen
static double footHeight = .0335;   //OP, spec
static const double kneeOffsetX = .0;     //OP
static double dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
static double aThigh = atan(kneeOffsetX/thighLength);
static double dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX);
static double aTibia = atan(kneeOffsetX/tibiaLength);

namespace ZMPWalking {

void set_long_leg_adjusted_values(double thigh, double tibia, double hip_y_offset, double hip_z_offset, double foot_height) {
    thighLength = thigh;
    tibiaLength = tibia;
    hipOffsetY = hip_y_offset;
    hipOffsetZ = hip_z_offset;
    footHeight = foot_height;
    dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX);
    aThigh = atan(kneeOffsetX/thighLength);
    dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX);
    aTibia = atan(kneeOffsetX/tibiaLength);
}

class Transform {
private:
Eigen::Matrix4d t;
public:
    Transform():t(Eigen::Matrix4d::Zero()){t(3,3) = 1;} //TODO Richtig initialsieren
    Transform(const Eigen::Matrix4d& m):t(m){}
    ~Transform() {}

    inline Transform& rotateX(double a = 0);
    inline Transform& rotateY(double a = 0);
    inline Transform& rotateZ(double a = 0);
    inline double& operator() (int i, int j){
        return t(i,j);
    }
    const double& operator() (int i, int j) const{
        return t(i,j);
    }
    void apply(Eigen::Vector3d& v);
    inline Transform operator * (const Transform& t1) const;
    inline Transform inv() const;
};

} //namespace

using ZMPWalking::Transform;

//Vorausdeklaration
inline Transform transform6D(const Eigen::Matrix<double, 6, 1>& p);
inline Eigen::Matrix<double, 6, 1>
darwinop_kinematics_inverse_lleg(const Transform& trLeg);
inline Eigen::Matrix<double, 6, 1>
darwinop_kinematics_inverse_rleg(const Transform& trLeg);
inline Eigen::Matrix<double, 6, 1>
darwinop_kinematics_inverse_leg(const Transform& trLeg);

inline
Transform Transform::operator* (const Transform &t1) const{
    //return Transform(t * t1.t);//Matrixmultiplikation
  Transform tr;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      tr(i,j) = t(i,0)*t1(0,j) + t(i,1)*t1(1,j) +
      t(i,2)*t1(2,j) + t(i,3)*t1(3,j);
    }
  }
  return tr;
}

inline
Transform Transform::inv () const{
    Transform tr;
    // Transpose rotation:
    tr.t.topLeftCorner<3, 3>() = t.topLeftCorner<3, 3>().transpose();
    // Compute inv translation:
    tr.t.col(3).head<3>() -= tr.t.topLeftCorner<3, 3>() * t.col(3).head<3>(); // TODO nicht ganz das Gleiche zu TeamDarwin
    return tr;
}

Eigen::Matrix<double, 12, 1> ZMPWalking::inverse_legs(
                const Eigen::Matrix<double, 6, 1>& pLLeg,
                const Eigen::Matrix<double, 6, 1>& pRLeg,
                const Eigen::Matrix<double, 6, 1>& pTorso)
{
    //std::cout<< "Inverse Legs" << std::endl;
    Transform trLLeg = transform6D(pLLeg);
    Transform trRLeg = transform6D(pRLeg);
    Transform trTorso = transform6D(pTorso);

    Transform invTrTorso = trTorso.inv();

    Transform trTorso_LLeg = invTrTorso*trLLeg;
    Transform trTorso_RLeg = invTrTorso*trRLeg;

    Eigen::Matrix<double, 12, 1> r;
    //The first 6 values of r represent the left leg, the last 6 values the right one
    r.head<6>() = darwinop_kinematics_inverse_lleg(trTorso_LLeg);
    r.tail<6>() = darwinop_kinematics_inverse_rleg(trTorso_RLeg);
    //Manuelles swappen, da die Originaldaten das so wollen -,-
    //double tmp;
    //tmp = r(0); r(0) = -r(1); r(1) = tmp;
    //tmp = r(6); r(6) = -r(7); r(7) = tmp;
    return r;
}

inline
Transform transform6D(const Eigen::Matrix<double, 6, 1>& p) {
    Eigen::Matrix4d m(Eigen::Matrix4d::Identity());
    //Transformiert von Euler Winkel bzw. (x,y,z,Pitch,Roll,Yaw) in Homogene Koordinaten ..
    // siehe auch https://de.wikipedia.org/wiki/Eulerwinkel#.E2.80.9Ez.2C_y.E2.80.B2.2C_x.E2.80.B3-Konvention.E2.80.9C_in_der_Fahrzeugtechnik
    double cwx = cos(p(3));
    double swx = sin(p(3));
    double cwy = cos(p(4));
    double swy = sin(p(4));
    double cwz = cos(p(5));
    double swz = sin(p(5));
    m.row(0).head<3>()<< cwy*cwz,    swx*swy*cwz-cwx*swz,   cwx*swy*cwz+swx*swz;
    m.row(1).head<3>()<< cwy*swz,    swx*swy*swz+cwx*cwz,   cwx*swy*swz-swx*cwz;
    m.row(2).head<3>()<< -swy   ,    swx*cwy            ,   cwx*cwy;

    m.col(3).head<3>() = p.head<3>();
    return Transform(m);
}
/**
* Wendet die Homogene Transformation (an der die Methode aufgerufen wird) auf den Vector v an und verändert ihn.
* Erst wir der Vector durch die 4 Zeile Bewegt (Translation) und dann Rotiert,.
*/
inline void Transform::apply(Eigen::Vector3d& v) {
    v = t.col(3).head<3>() + t.topLeftCorner<3, 3>() * v;
}

inline Transform& Transform::rotateX(double a) {
  //  t*=Robot::Kinematics::pitch(a);
    double ca = cos(a);
    double sa = sin(a);
    for (int i = 0; i < 3; i++) { //TODO Muss hier nicht i=1  & Gespiegelt? & Was ist mit den Translationswerten
        double ty = t(i, 1);
        double tz = t(i, 2);
        t(i, 1) = ca*ty + sa*tz;
        t(i, 2) = -sa*ty + ca*tz;
    }
    return *this;
}

inline Transform& Transform::rotateY(double a) { // TODO muss hier nicht 0 und 2 explizit
//    t*=Robot::Kinematics::yaw(a);
    double ca = cos(a);
    double sa = sin(a);
    for (int i = 0; i < 3; i++) {
        double tx = t(i, 0);
        double tz = t(i, 2);
        t(i, 0) = ca*tx - sa*tz;
        t(i, 2) = sa*tx + ca*tz;
    }
    return *this;
}

inline Transform& Transform::rotateZ(double a) { //TODO muss hier nicht i < 2
    //t*=Robot::Kinematics::roll(a);
    double ca = cos(a);
    double sa = sin(a);
    for (int i = 0; i < 3; i++) {
        double tx = t(i, 0);
        double ty = t(i, 1);
        t(i, 0) = ca*tx + sa*ty;
        t(i, 1) = -sa*tx + ca*ty;
  }
    return *this;
}

inline
Eigen::Matrix<double, 6, 1>
darwinop_kinematics_inverse_leg(
            const Transform& trLeg,
            int leg)
{
    Eigen::Matrix<double, 6, 1> qLeg;
    bool left = (leg == LEG_LEFT); // Left leg

    Transform trInvLeg = trLeg.inv();

    // Hip Offset vector in Torso frame
    Eigen::Vector3d xHipOffset;
    if (left) {
        xHipOffset = Eigen::Vector3d(0, hipOffsetY, -hipOffsetZ);
    }
    else {
        xHipOffset = Eigen::Vector3d(0, -hipOffsetY, -hipOffsetZ);
    }

    // Hip Offset in Leg frame
    Eigen::Vector3d xLeg = xHipOffset;
    trInvLeg.apply(xLeg);
    xLeg(2) -= footHeight;//??

    // Knee pitch
    double dLeg = xLeg.squaredNorm();

    double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);
    if (cKnee > 1) cKnee = 1;
    if (cKnee < -1) cKnee = -1;
    double kneePitch = acos(cKnee);

    // Angle pitch and roll
    double ankleRoll = atan2(xLeg(1), xLeg(2));
    double lLeg = sqrt(dLeg);
    if (lLeg < 1e-16) lLeg = 1e-16;
    double pitch0 = asin(dThigh*sin(kneePitch)/lLeg);
    double anklePitch = asin(-xLeg(0)/lLeg) - pitch0;

    Transform rHipT = trLeg;
    rHipT = rHipT.rotateX(-ankleRoll).rotateY(-anklePitch-kneePitch);

    double hipYaw = atan2(-rHipT(0,1), rHipT(1,1));
    double hipRoll = asin(rHipT(2,1));
    double hipPitch = atan2(-rHipT(2,0), rHipT(2,2));

    // Need to compensate for KneeOffsetX:
    qLeg << hipYaw, hipRoll, hipPitch-aThigh, kneePitch+aThigh+aTibia, anklePitch-aTibia, ankleRoll;
    return qLeg;

}

inline
Eigen::Matrix<double, 6, 1>
darwinop_kinematics_inverse_lleg(const Transform& trLeg)
{
    return darwinop_kinematics_inverse_leg(trLeg, LEG_LEFT);
}

inline
Eigen::Matrix<double, 6, 1>
darwinop_kinematics_inverse_rleg(const Transform& trLeg)
{
    return darwinop_kinematics_inverse_leg(trLeg, LEG_RIGHT);
}

