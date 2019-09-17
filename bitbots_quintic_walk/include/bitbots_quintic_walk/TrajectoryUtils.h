/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef TRAJECTORYUTILS_H
#define TRAJECTORYUTILS_H

#include "bitbots_splines/SmoothSpline.hpp"
#include "bitbots_splines/SplineContainer.hpp"
#include <Eigen/Dense>

namespace bitbots_quintic_walk {

/**
 * Simple typedef for trajectories container
 */
typedef bitbots_splines::SplineContainer<bitbots_splines::SmoothSpline> Trajectories;

/**
 * Return initialized trajectories for
 * trunk/foot ik cartesian with empty splines
 */
Trajectories trajectoriesInit();

/**
 * Compute from given spline container 
 * trajectory Cartesian trunk and foot
 * position/velocity/acceleration 
 * and assign it to given vector
 */
void trajectoriesTrunkFootPos(
    double t, const Trajectories &traj,
    Eigen::Vector3d &trunk_pos,
    Eigen::Vector3d &trunk_axis,
    Eigen::Vector3d &foot_pos,
    Eigen::Vector3d &foot_axis);
void trajectoriesTrunkFootVel(
    double t, const Trajectories &traj,
    Eigen::Vector3d &trunk_pos_vel,
    Eigen::Vector3d &trunk_axis_vel,
    Eigen::Vector3d &foot_pos_vel,
    Eigen::Vector3d &foot_axis_vel);
void trajectoriesTrunkFootAcc(
    double t, const Trajectories &traj,
    Eigen::Vector3d &trunk_pos_acc,
    Eigen::Vector3d &trunk_axis_acc,
    Eigen::Vector3d &foot_pos_acc,
    Eigen::Vector3d &foot_axis_acc);
void trajectoriesSupportFootState(
    double t, const Trajectories &traj,
    bool &isDoubleSupport,
    bool &isLeftsupportFoot);

/**
 * Default Cartesian state check function.
 * Return positive cost value
 * if given time and Cartesian state are outside
 * standard valid range
 */
double DefaultCheckState(
    const Eigen::VectorXd &params,
    double t,
    const Eigen::Vector3d &trunkPos,
    const Eigen::Vector3d &trunkAxis,
    const Eigen::Vector3d &footPos,
    const Eigen::Vector3d &footAxis);

}

#endif

