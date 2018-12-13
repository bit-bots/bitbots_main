/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#ifndef AXISANGLE_H
#define AXISANGLE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdexcept>

namespace bitbots_splines {

/**
 * Axis angle representation. 
 * The direction is the rotation axis and 
 * the norm is the angle in radian.
 */

/**
 * Compute and return the rotation matrix with given
 * axis. Identity is returned if null vector is given.
 * Rotation angle have to be in -M_PI/2.0:M_PI/2.0
 */
inline Eigen::Matrix3d AxisToMatrix(const Eigen::Vector3d& axis)
{
    double theta = axis.norm();
    if (fabs(theta) > M_PI/2.0) {
        throw std::logic_error("AxisAngle unbounded angle (in -M_PI/2:M_PI/2)");
    }
    if (theta <= 0.0) {
        return Eigen::Matrix3d::Identity();
    } else {
        Eigen::Vector3d vect = axis.normalized();
        Eigen::AngleAxisd axisAngle(theta, vect);
        return axisAngle.matrix();
    }
}

/**
 * Convert and return the rotation axis vector from
 * given rotation matrix.
 * Null vector is returned if identity matrix is given.
 * No check is done on input matrix format.
 */
inline Eigen::Vector3d MatrixToAxis(const Eigen::Matrix3d& mat)
{
    //Skew is the anti-symetric matrix
    Eigen::Matrix3d skew = mat - mat.transpose();
    //Rotation axis extraction
    Eigen::Vector3d axis;
    axis(0) = skew(2, 1);
    axis(1) = skew(0, 2);
    axis(2) = skew(1, 0);
    //Compute rotation angle        
    if (axis.norm() > 0.00001) {
        double theta = std::asin(axis.norm()/2.0);
        return theta*axis.normalized();
    } else {
        return Eigen::Vector3d(0.0, 0.0, 0.0);
    }
}

/**
 * Conversion from rotation axis 
 * (first or second) differential to actual 
 * angular velocity or acceleration in world frame
 * Reference:
 * Representing attitude: Euler angles, unit quaternions, and rotation vectors
 * Diebel 2006 
 * Page 21 (eq. 259)
 * Axis: world to moving frame rotation axis (rotation vector)
 * AxisDiff: time differential of axis (rotation vector rate)
 * Returns rotation velocity or acceleration of moving frame 
 * in world frame (angular velocity/acceleration)
 */
inline Eigen::Vector3d AxisDiffToAngularDiff(
    const Eigen::Vector3d& axis, const Eigen::Vector3d& axisDiff)
{
    double v = axis.norm();
    double v1 = axis(0);
    double v2 = axis(1);
    double v3 = axis(2);
    double cv2 = cos(v/2.0);
    double sv2 = sin(v/2.0);
    double a = cv2*v - 2.0*sv2;
    //Check for identity rotation
    if (v < 0.00001) {
        return axisDiff;
    }

    //Eq. 237
    Eigen::MatrixXd W(3, 4);
    W <<
        -v1*sv2, v*cv2,   -v3*sv2, v2*sv2,
        -v2*sv2, v3*sv2,  v*cv2,   -v1*sv2,
        -v3*sv2, -v2*sv2, v1*sv2,  v*cv2;
    W *= 1.0/v;
    //Eq. 214
    Eigen::MatrixXd G(4, 3);
    G <<
        -v1*v*v*sv2,           -v2*v*v*sv2,           -v3*v*v*sv2,
        2.0*v*v*sv2 + v1*v1*a, v1*v2*a,               v1*v3*a,
        v1*v2*a,               2.0*v*v*sv2 + v2*v2*a, v2*v3*a,
        v1*v3*a,               v2*v3*a,               2.0*v*v*sv2 + v3*v3*a;
    G *= 1.0/(2.0*v*v*v);

    //Eq. 259
    return 2.0*W*G*axisDiff;
}

}

#endif

