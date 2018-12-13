/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef TRAJECTORYUTILS_H
#define TRAJECTORYUTILS_H

#include "SmoothSpline.hpp"
#include "SplineContainer.hpp"
#include <Eigen/Dense>


namespace bitbots_splines {

/**
 * Simple typedef for trajectories container
 */
typedef SplineContainer<SmoothSpline> Trajectories;

/**
 * Return initialized trajectories for
 * trunk/foot ik cartesian with empty splines
 */
Trajectories TrajectoriesInit();

/**
 * Compute from given spline container 
 * trajectory Cartesian trunk and foot
 * position/velocity/acceleration 
 * and assign it to given vector
 */
void TrajectoriesTrunkFootPos(
    double t, const Trajectories& traj,
    Eigen::Vector3d& trunkPos,
    Eigen::Vector3d& trunkAxis,
    Eigen::Vector3d& footPos,
    Eigen::Vector3d& footAxis);
void TrajectoriesTrunkFootVel(
    double t, const Trajectories& traj,
    Eigen::Vector3d& trunkPosVel,
    Eigen::Vector3d& trunkAxisVel,
    Eigen::Vector3d& footPosVel,
    Eigen::Vector3d& footAxisVel);
void TrajectoriesTrunkFootAcc(
    double t, const Trajectories& traj,
    Eigen::Vector3d& trunkPosAcc,
    Eigen::Vector3d& trunkAxisAcc,
    Eigen::Vector3d& footPosAcc,
    Eigen::Vector3d& footAxisAcc);
void TrajectoriesSupportFootState(
    double t, const Trajectories& traj,
    bool& isDoubleSupport, 
    bool& isLeftsupportFoot);


/**
 * Default Cartesian state check function.
 * Return positive cost value
 * if given time and Cartesian state are outside
 * standard valid range
 */
double DefaultCheckState(
    const Eigen::VectorXd& params,
    double t,
    const Eigen::Vector3d& trunkPos,
    const Eigen::Vector3d& trunkAxis,
    const Eigen::Vector3d& footPos,
    const Eigen::Vector3d& footAxis);


}

#endif

