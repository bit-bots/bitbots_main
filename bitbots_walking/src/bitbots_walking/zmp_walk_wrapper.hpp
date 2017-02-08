
#include "bitbots_common/pose/pose.hpp"
#include "body.hpp"

#include "zmp_math_basics.h"
#include "zmp_walk_parameter.hpp"

namespace ZMPWalking{

class ZMPWalkWrapper;

ZMPWalkWrapper* getInstance(const ZMPParameter& parameter);

class ZMPWalkWrapper{
public:
    typedef ZMPFootPhaseDefinition::FootPhase FootPhase;
    ZMPWalkWrapper(){}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual double get_hip_pitch() = 0;
    virtual Robot::Pose& get_pose()= 0;
    virtual double get_l_shoulder_pitch_offset()= 0;
    virtual double get_r_shoulder_pitch_offset()= 0;
    virtual double  get_l_shoulder_roll_offset()= 0;
    virtual double get_r_shoulder_roll_offset()= 0;
    virtual double get_hip_pitch_offset()= 0;
    virtual std::string get_robottype()= 0;
    virtual const Eigen::Vector3d& get_uLeft()= 0;
     virtual const Eigen::Vector3d& get_uRight()= 0;
    virtual const Eigen::Vector3d& get_velocity()= 0;
     virtual bool is_active()= 0;
    virtual void set_active(bool active)= 0 ;
    virtual void set_hip_pitch(double hip_pitch)= 0;
    virtual void set_l_shoulder_pitch_offset(double offset)= 0;
    virtual void set_r_shoulder_pitch_offset(double offset)= 0;
    virtual void set_l_shoulder_roll_offset(double offset)= 0;
    virtual void set_r_shoulder_roll_offset(double offset)= 0;
    virtual void set_hip_pitch_offset(double offset)= 0;
    virtual void set_gyro_data(const Eigen::Vector3d& gyro)= 0;
    virtual void set_long_legs(double thigh, double tibia, double hip_y_offset, double hip_z_offset, double foot_height)= 0;
    virtual void set_belly_roll_pid(float p, float i, float d)= 0;
    virtual void set_velocity(double tvx, double vy, double va)= 0;
    virtual void stance_reset()= 0;
    virtual void start()= 0;
    virtual void stop()= 0;
    virtual FootPhase update()= 0;
    virtual void set_robottype(std::string robottype)= 0;
};
}