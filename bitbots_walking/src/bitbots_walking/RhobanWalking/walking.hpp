#ifndef _Rhoban_WALK_HPP
#define _Rhoban_WALK_HPP
#include "IKWalk.hpp"
#include "bitbots_common/pose/pose.hpp"
#include "../zmp_math_basics.h"

class RhobanWalk{
public:
    RhobanWalk();
    typedef ZMPFootPhaseDefinition::FootPhase FootPhase;

    void start();
    void stop(){stopRequest = 1;}
    FootPhase update();
    Robot::Pose& get_pose(){ return  m_pose;}

    Rhoban::IKWalkParameters& getParams()
    {
        return params;
    }
    void set_frequenzy(double freq);

    void set_velocity(double vx, double vy, double va);
    void set_gyro(double gx, double gy, double ga)
    {
    gx=gx; gyroY = gy; ga=ga;
    }

    bool is_active(){
        return m_active; //return stopRequest < 10;
         }

private:
    bool m_active = false;
    struct Rhoban::IKWalkParameters params;

    struct Rhoban::IKWalkOutputs outputs;
    double phase;
    double engineFrequenzy = 100;
    Robot::Pose m_pose;
    int stopRequest = 0;
    double goalValX = 0, goalValY = 0, goalValA = 0;
    double gyroY = 0;
    double trunkXOffset_backUp, dynamicTrunkOffset = 0 ;
    void updateVelocity();
    void updateGyro();
};

#endif