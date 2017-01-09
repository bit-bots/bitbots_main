#ifndef _ZMP_BODY_HPP_
#define _ZMP_BODY_HPP_

#include <Eigen/Core>
#include <cmath>
#include "pose.hpp"
#include "zmp_math_basics.h"
#include <iostream>

namespace ZMPWalking{

static inline double trim_to_range(double f){
    if(f > 180) {f -= 360 * ((long)((f+180)/360));std::cout<<"Wuahhhh!!!!!!!!!"<<f<<std::endl;}
    //fmod()??
    if(f < -180){f += 360 * ((long)((f-180)/360));std::cout<<"Wuahhhh!!!!!!!!!"<<f<<std::endl;}
    return f;
}

class Body{
private:
Robot::Pose m_pose;

double m_l_shoulder_pitch_offset, m_r_shoulder_pitch_offset, m_l_shoulder_roll_offset, m_r_shoulder_roll_offset, m_hip_pitch_offset;
bool m_hip_yaws_switced;

int m_pDefault;

public:

    enum LegMapping{RHIP_YAW=6, RHIP_ROLL=7, RHIP_PITCH=8, RKNEE=9, RANKLE_PITCH=10, RANKLE_ROLL=11,
        LHIP_YAW=0, LHIP_ROLL=1, LHIP_PITCH=2, LKNEE=3, LANKLE_PITCH=4, LANKLE_ROLL=5};

    Body(int pDefault) {
        m_pDefault = pDefault;
        m_l_shoulder_pitch_offset = m_r_shoulder_pitch_offset = m_hip_pitch_offset = 0;
        m_l_shoulder_roll_offset = m_r_shoulder_roll_offset = m_hip_pitch_offset = 0;
        m_hip_yaws_switced = true;
    }

    void set_l_shoulder_pitch_offset(double offset) {
        m_l_shoulder_pitch_offset = offset;
    }

    void set_r_shoulder_pitch_offset(double offset) {
        m_r_shoulder_pitch_offset = offset;
    }

    void set_l_shoulder_roll_offset(double offset) {
        m_l_shoulder_roll_offset = offset;
    }

    void set_r_shoulder_roll_offset(double offset) {
        m_r_shoulder_roll_offset = offset;
    }

    void set_hip_pitch_offset(double offset) {
        m_hip_pitch_offset = offset;
    }

    void toggle_switched_hip_yaws(bool switched) {
        m_hip_yaws_switced = switched;
    }

    double get_l_shoulder_pitch_offset()
    {
        return m_l_shoulder_pitch_offset;
    }

    double get_r_shoulder_pitch_offset()
    {
        return m_r_shoulder_pitch_offset;
    }

    double get_l_shoulder_roll_offset()
    {
        return m_l_shoulder_roll_offset;
    }

    double get_r_shoulder_roll_offset()
    {
        return m_r_shoulder_roll_offset;
    }

    double get_hip_pitch_offset()
    {
        return m_hip_pitch_offset;
    }

    Robot::Pose& get_pose(){
        return m_pose;
    }
    //schuss pose schleift stark mit den armen bei wheatly
    void set_larm_command(const Eigen::Vector3d& qLArmActual){
        m_pose.get_l_shoulder_yaw().set_goal(0);
        m_pose.get_l_shoulder_pitch().set_goal(trim_to_range(qLArmActual(0)  * rad_to_degree - 90 - m_l_shoulder_pitch_offset));
        m_pose.get_l_shoulder_roll().set_goal(trim_to_range(qLArmActual(1) * rad_to_degree + 30 - m_l_shoulder_roll_offset));
        m_pose.get_l_elbow().set_goal(trim_to_range(qLArmActual(2) * rad_to_degree + 90));
    }

    void set_larm_command(const Eigen::Vector2d& qLArmActual){
        m_pose.get_l_shoulder_yaw().set_goal(0);
        m_pose.get_l_shoulder_pitch().set_goal(trim_to_range(qLArmActual(0)  * rad_to_degree - 90 - m_l_shoulder_pitch_offset));
        m_pose.get_l_shoulder_roll().set_goal(trim_to_range(qLArmActual(1) * rad_to_degree + 30 - m_l_shoulder_roll_offset));
    }

    void set_rarm_command(const Eigen::Vector3d& qRArmActual){
        m_pose.get_r_shoulder_yaw().set_goal(0);
        m_pose.get_r_shoulder_pitch().set_goal(trim_to_range( - qRArmActual(0)  * rad_to_degree+ 90 + m_r_shoulder_pitch_offset));
        m_pose.get_r_shoulder_roll().set_goal(trim_to_range(qRArmActual(1) * rad_to_degree - 30 + m_r_shoulder_roll_offset));
        m_pose.get_r_elbow().set_goal(trim_to_range( - qRArmActual(2)  * rad_to_degree - 90));
    }

    void set_rarm_command(const Eigen::Vector2d& qRArmActual){
        m_pose.get_r_shoulder_yaw().set_goal(0);
        m_pose.get_r_shoulder_pitch().set_goal(trim_to_range( - qRArmActual(0)  * rad_to_degree+ 90 + m_r_shoulder_pitch_offset));
        m_pose.get_r_shoulder_roll().set_goal(trim_to_range(qRArmActual(1) * rad_to_degree - 30 + m_r_shoulder_roll_offset));
    }

    void set_belly_to_initial(float pitch=0.f, float roll=0.f) {
        m_pose.get_belly_pitch().set_goal(pitch*rad_to_degree);
        m_pose.get_belly_roll().set_goal(roll*rad_to_degree);
        m_pose.get_l_toe().set_goal(0.f);
        m_pose.get_r_toe().set_goal(0.f);
    }
    void set_belly_roll_pid(float p,float i, float d)
    {
    m_pose.get_belly_roll().set_p(p);
    m_pose.get_belly_roll().set_i(i);
    m_pose.get_belly_roll().set_d(d);
    }

    void set_lleg_command(const Eigen::Matrix<double, 12, 1>& qLegs){
        //m_pose.get_l_hip_yaw().set_goal(trim_to_range(qLegs(0)  * conversion));
        /*On Tamara motor 7 and 8 were switched, and tamara walked best of all our robots. Now we swap it in the code*/
        if(m_hip_yaws_switced)
            m_pose.get_l_hip_yaw().set_goal(trim_to_range(qLegs(RHIP_YAW)  * rad_to_degree));
        else
            m_pose.get_l_hip_yaw().set_goal(trim_to_range(qLegs(LHIP_YAW)  * rad_to_degree));
        m_pose.get_l_hip_roll().set_goal(trim_to_range( - qLegs(LHIP_ROLL) * rad_to_degree));
        m_pose.get_l_hip_pitch().set_goal(trim_to_range( - qLegs(LHIP_PITCH) * rad_to_degree + m_hip_pitch_offset));
        m_pose.get_l_knee().set_goal(trim_to_range( - qLegs(LKNEE) * rad_to_degree));
        m_pose.get_l_ankle_pitch().set_goal(trim_to_range(qLegs(LANKLE_PITCH) * rad_to_degree));
        m_pose.get_l_ankle_roll().set_goal(trim_to_range(qLegs(LANKLE_ROLL) * rad_to_degree));
        //m_pose.get_r_hip_yaw().set_goal(trim_to_range(qLegs(6) * conversion));
        /*On Tamara motor 7 and 8 were switched, and tamara walked best of all our robots. Now we swap it in the code*/
        if(m_hip_yaws_switced)
            m_pose.get_r_hip_yaw().set_goal(trim_to_range(qLegs(LHIP_YAW) * rad_to_degree));
        else
            m_pose.get_r_hip_yaw().set_goal(trim_to_range(qLegs(RHIP_YAW) * rad_to_degree));
        m_pose.get_r_hip_roll().set_goal(trim_to_range(- qLegs(RHIP_ROLL) * rad_to_degree));
        m_pose.get_r_hip_pitch().set_goal(trim_to_range(qLegs(RHIP_PITCH) * rad_to_degree - m_hip_pitch_offset));
        m_pose.get_r_knee().set_goal(trim_to_range(qLegs(RKNEE) * rad_to_degree));
        m_pose.get_r_ankle_pitch().set_goal(trim_to_range(-qLegs(RANKLE_PITCH) * rad_to_degree));
        m_pose.get_r_ankle_roll().set_goal(trim_to_range(qLegs(RANKLE_ROLL) * rad_to_degree));
    }

    void set_larm_hardness(double hardnessArm){
        int p = hardnessArm * m_pDefault;
        m_pose.get_l_shoulder_pitch().set_p(p);
        m_pose.get_l_shoulder_roll().set_p(p);
        m_pose.get_l_elbow().set_p(p);
    }

    void set_larm_hardness(const Eigen::Vector3d& hardnessArm){
        m_pose.get_l_shoulder_pitch().set_p(hardnessArm(0) * m_pDefault);
        m_pose.get_l_shoulder_roll().set_p(hardnessArm(1) * m_pDefault);
        m_pose.get_l_elbow().set_p(hardnessArm(2) * m_pDefault);
    }

    void set_lleg_hardness(double hardnessSupport){
        int p = hardnessSupport * m_pDefault;
        m_pose.get_l_hip_yaw().set_p(p);
        m_pose.get_l_hip_roll().set_p(p);
        m_pose.get_l_hip_pitch().set_p(p);
        m_pose.get_l_knee().set_p(p);
        m_pose.get_l_ankle_pitch().set_p(p);
        m_pose.get_l_ankle_roll().set_p(p);
    }

    void set_rarm_hardness(double hardnessArm){
        int p = hardnessArm * m_pDefault;
        m_pose.get_r_shoulder_pitch().set_p(p);
        m_pose.get_r_shoulder_roll().set_p(p);
        m_pose.get_r_elbow().set_p(p);
    }

    void set_rarm_hardness(const Eigen::Vector3d& hardnessArm){
        m_pose.get_r_shoulder_pitch().set_p(hardnessArm(0) * m_pDefault);
        m_pose.get_r_shoulder_roll().set_p(hardnessArm(1) * m_pDefault);
        m_pose.get_r_elbow().set_p(hardnessArm(2) * m_pDefault);
    }

    void set_rleg_hardness(double hardnessSwing){
        int p = hardnessSwing * m_pDefault;
        m_pose.get_r_hip_yaw().set_p(p);
        m_pose.get_r_hip_roll().set_p(p);
        m_pose.get_r_hip_pitch().set_p(p);
        m_pose.get_r_knee().set_p(p);
        m_pose.get_r_ankle_pitch().set_p(p);
        m_pose.get_r_ankle_roll().set_p(p);

    }

};

}//namespace

#endif
