#ifndef ZMP_INVERSE_LEGS_HPP_
#define ZMP_INVERSE_LEGS_HPP_

#include <cmath>
#include <Eigen/Core>

namespace ZMPWalking{

    void set_long_leg_adjusted_values(double thigh, double tibia, double hip_y_offset, double hip_z_offset, double foot_height);

Eigen::Matrix<double, 12, 1> inverse_legs(
                const Eigen::Matrix<double, 6, 1>& pLLeg,
                const Eigen::Matrix<double, 6, 1>& pRLeg,
                const Eigen::Matrix<double, 6, 1>& pTorso);

}//namespace

#endif
