#ifndef _KINEMATIK_BASICS_HPP_
#define _KINEMATIK_BASICS_HPP_

#include <Eigen/Core>
#include <cmath>

namespace Robot {
namespace Kinematics{

template<class TYPE>
inline Eigen::Matrix<TYPE, 4, 4> pitch(TYPE angle) {
    TYPE c = std::cos(angle);
    TYPE s = std::sin(angle);
    Eigen::Matrix<TYPE, 4, 4> result;
    result << 1, 0, 0, 0,
              0, c,-s, 0,
              0, s, c, 0,
              0, 0, 0, 1;

    return result;
}

template<class TYPE>
inline Eigen::Matrix<TYPE, 4, 4> yaw(TYPE angle) {
    TYPE c = std::cos(angle);
    TYPE s = std::sin(angle);
    Eigen::Matrix<TYPE, 4, 4> result;
    result << c,  0,  s, 0,
              0,  1,  0, 0,
             -s,  0,  c, 0,
              0,  0,  0, 1;

    return result;
}

template<class TYPE>
inline Eigen::Matrix<TYPE, 4, 4> roll(TYPE angle) {
    TYPE c = std::cos(angle);
    TYPE s = std::sin(angle);
    Eigen::Matrix<TYPE, 4, 4> result;
    result << c, -s,  0, 0,
              s,  c,  0, 0,
              0,  0,  1, 0,
              0,  0,  0, 1;

    return result;
}

inline
Eigen::Matrix4d translate(double x, double y, double z) {
    Eigen::Matrix4d result;
    result << 1, 0, 0, x,
    0, 1, 0, y,
    0, 0, 1, z,
    0, 0, 0, 1;
    return result;
}


} } // namespace
#endif
