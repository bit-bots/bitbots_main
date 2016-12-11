#ifndef INVERSE_COMPUTATION_HPP__
#define INVERSE_COMPUTATION_HPP__

#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>

#include "ros/console.h"
#include "kinematic_robot.hpp"

namespace Robot {
namespace Kinematics {
namespace _intern {
//enum N{jacobi_max = ::Robot::Kinematics::Robot::TemplateNumbers::jacobi_max};
//typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor, jacobi_max, jacobi_max> MatrixType;
typedef Eigen::MatrixXd MatrixType;
typedef typename KRobot::InverseJacobiType MatrixReturnType;
typedef typename KRobot::JacobiType JacobiType;
using Eigen::MatrixBase;

namespace __intern {
/**
 * Computes the inverse of a matrix, when the check can be performed, this method returns true, false otherwise. Success will be false, when the matrix is singlular.
 * When no check can be performed, then success is always true
 *
 * \param to_inverse: the matrix to be inverted
 * \param inverse: the matrix, the result will be stored in
 * \param success: the computation result, if accessible, false otherwise
 * \return true, when a check is possible, false otherwise
 */
template<class Derived>
BITBOTS_INLINE bool inverse_small_matrix_with_check(const MatrixBase<Derived>& to_inverse, MatrixBase<Derived>& inverse, bool& success) {
    #define MATRIX_COLS(TYPE, VAR) ((TYPE::ColsAtCompileTime == TYPE::MaxColsAtCompileTime && TYPE::ColsAtCompileTime!= Eigen::Dynamic)? TYPE::ColsAtCompileTime: VAR.cols())
    if(MATRIX_COLS(Derived, to_inverse) <= 4){
        switch(MATRIX_COLS(Derived, to_inverse)) {
            #undef MATRIX_COLS
            #define SWITCH_CASE_SPECIALIZATION(NUM) \
            case(NUM):{ \
                const Eigen::Matrix<double, NUM, NUM> m(to_inverse.template block<NUM, NUM>(0, 0)); \
                Eigen::Matrix<double, NUM, NUM> inv; \
                inverse_small_matrix_with_check<Eigen::Matrix<double, NUM, NUM> >(m, inv, success); \
                if(inverse.cols() == 0) { \
                    inverse = Derived(NUM, NUM); \
                } \
                inverse.template block<NUM, NUM>(0, 0) = inv; \
                return true; \
            }
            SWITCH_CASE_SPECIALIZATION(1)
            SWITCH_CASE_SPECIALIZATION(2)
            SWITCH_CASE_SPECIALIZATION(3)
            SWITCH_CASE_SPECIALIZATION(4)
            #undef SWITCH_CASE_SPECIALIZATION
            default: {
                success = true;
                inverse = to_inverse.inverse();
                throw std::runtime_error("This case should not be reachable, enable the SWITCH_CASE_SPECIALIZATION to void this error");
                return false;
            }
        }
    } else {
        success = false;
        //inverse = to_inverse.inverse();
        return false;
    }
}

#define INVERSE_WITH_CHECK_SPECIALIZATION(NUM) \
template<> \
BITBOTS_INLINE bool inverse_small_matrix_with_check(const MatrixBase<Eigen::Matrix<double, NUM, NUM> >& to_inverse, MatrixBase<Eigen::Matrix<double, NUM, NUM> >& inverse, bool& success) { \
    ((Eigen::Matrix<double, NUM, NUM>&)to_inverse).computeInverseWithCheck(((Eigen::Matrix<double, NUM, NUM>&)inverse), success); \
    return true; \
}

INVERSE_WITH_CHECK_SPECIALIZATION(1)
INVERSE_WITH_CHECK_SPECIALIZATION(2)
INVERSE_WITH_CHECK_SPECIALIZATION(3)
INVERSE_WITH_CHECK_SPECIALIZATION(4)
#undef INVERSE_WITH_CHECK_SPECIALIZATION

static const unsigned epsilon_increase_factor = 100;
static const double default_epsilon = 1e-1;
static const unsigned max_tries=1;

/**
 * The pseudo inverse to be applied on the right side
 * \param j: jacobi matrix to be inverted
 * \param epsilon: an error value added on the diagonal, when the matrix is not invertible to fix this issue
 */
static BITBOTS_INLINE MatrixReturnType pseudo_inverse_r(const JacobiType& j, const Eigen::MatrixXd* N, double epsilon=default_epsilon) {
    typedef Eigen::MatrixXd Matrix;
    const Matrix jjt = j * (*N) * j.transpose();
    Matrix inverse;
    bool success;
    inverse_small_matrix_with_check<Matrix>(jjt, inverse, success);
    if(!success) {
        return j.transpose() * (jjt + epsilon * Matrix::Identity(jjt.cols(), jjt.cols())).inverse();
    }
    unsigned tries = 0;
    while(!success && tries < max_tries) {
        Matrix to_inverse(jjt + epsilon * Matrix::Identity(jjt.cols(), jjt.rows()));
        inverse_small_matrix_with_check<Matrix>(to_inverse, inverse, success);
        ++tries;
        epsilon *= epsilon_increase_factor;
    }
    //ROS_DEBUG_STREAM(if(tries != 0)std::cout<<"Inverse not computed on first try: "<<tries<<std::endl);
    return j.transpose() * inverse;
}

/**
 * The pseudo inverse to be applied on the left side
 * \param j: jacobi matrix to be inverted
 * \param epsilon: an error value added on the diagonal, when the matrix is not invertible to fix this issue
 */
static BITBOTS_INLINE MatrixReturnType pseudo_inverse_l(const JacobiType& j, double epsilon=default_epsilon) {
    const MatrixType jtj = j.transpose() * j;
    // Matrixinversion with check is only provided for fixed size matrices having not more than 4 cols
    MatrixType inverse;
    bool success;
    if(! inverse_small_matrix_with_check<MatrixType>(jtj, inverse, success)){
        //std::cerr<<"This kind of matix pseudo inversion is not recommended "<< __FILE__<< ": " << __LINE__<<std::endl;
        return (jtj + epsilon * MatrixType::Identity(jtj.cols(), jtj.cols())).inverse() * j.transpose();
    }
    unsigned tries = 0;
    while(!success && tries < max_tries) {
        MatrixType to_inverse(jtj + epsilon * MatrixType::Identity(jtj.cols(), jtj.cols()));
        epsilon *= epsilon_increase_factor;
        ++tries;
        inverse_small_matrix_with_check<MatrixType>(to_inverse, inverse, success);
    }
    //ROS_DEBUG_STREAM(if(tries != 0)std::cout<<"Inverse not computed on first try: "<<tries<<std::endl);
    return inverse * j.transpose();
}

static BITBOTS_INLINE MatrixReturnType inverse(const JacobiType& j, double epsilon=default_epsilon) {
    MatrixType inverse;
    bool success = false;
    if(! inverse_small_matrix_with_check<JacobiType>(j, inverse, success)) {
        return (j + epsilon * JacobiType::Identity(j.cols(), j.cols())).inverse();
    }
    unsigned tries = 0;
    while(!success && tries < max_tries) {
        inverse_small_matrix_with_check<JacobiType>(j, inverse, success);
        ++tries;
    }
    return inverse;
}

} //namespace __intern

/**
 * This method computes a pseudo inverse to a given matrix. This method tries to be as accurate as possible, so it distinguishes
 * wheater it computes the inverse matrix of mmt or mtm. Finally the original matrix is applied to a fitting site, so that this
 * inverse is closest to the inverse, when mmt or mtm is invertible.
 * \param j: The jacobi matrix to be inverted
 * \param epsilon: An optional error value to avoid singularity when inverting jacobi matrices with a rang lower 3
 */
template<bool minimal_matrix=false>
static BITBOTS_INLINE MatrixReturnType pseudo_inverse(const JacobiType& j, const Eigen::MatrixXd* N = nullptr, double epsilon=__intern::default_epsilon) {
    if(!N && ! minimal_matrix) {
        unsigned begin = 0, end = j.cols() - 1;
        while(end != 0 && j.col(end) == Eigen::VectorXd::Zero(j.rows())) --end;
        if(end == 0 && j.col(end) == Eigen::VectorXd::Zero(j.rows()))
            throw std::runtime_error("Inversion of empty matrix");
        while(j.col(begin) == Eigen::VectorXd::Zero(j.rows())) ++begin;
        if(begin || end != (unsigned)j.cols() - 1) {
            return (MatrixReturnType(j.cols(), j.rows())<<Eigen::MatrixXd::Zero(begin, j.rows()), pseudo_inverse<true>(j.block<Eigen::Dynamic, Eigen::Dynamic>(0, begin, j.rows(), end - begin + 1), N, epsilon), Eigen::MatrixXd::Zero(j.cols() - end  - 1, j.rows())).finished();
        }
    }
    if(!N && j.cols() < j.rows()) {
        return __intern::pseudo_inverse_l(j, epsilon);
    } else if(!N && j.cols() == j.rows()) {
        return __intern::inverse(j, epsilon);
    } else {
        return __intern::pseudo_inverse_r(j, N ,epsilon);
    }
}

} } }//namespace _intern

#endif
