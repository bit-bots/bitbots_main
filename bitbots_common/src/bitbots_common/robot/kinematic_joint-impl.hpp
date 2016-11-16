#ifdef NEW_KINAMATIC_JOINT_IMPLEMENTATION_HPP__
    #error "This class must not be included twice"
#else
    #define NEW_KINAMATIC_JOINT_IMPLEMENTATION_HPP__
#endif

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "../debug/debugmacro.h"

#include "kinematic_joint.hpp"

#if !defined ROBOT_IMPL_FOR_JOINT_USAGE && !defined INCLUDE_JOINT_IMPL_FOR_TESTS
    #error "Joint Implementation should only be included from robot.cpp"
#endif

namespace Robot {
namespace Kinematics {

typedef unsigned int uint;
typedef KJoint::AxisType AxisType;

/**
 * This method creates a rotation matrix from a given vector representing the angles
 * Axis definition:
 * x: x-axis goes front
 * y: y-axis goes left
 * z: z-goes above
 * \param rotations: The rotations around the given axis x, y and z
 */
static BITBOTS_INLINE Eigen::Matrix3d create_rotation_matrix(const Eigen::Vector3d& rotations)
{
    typedef Eigen::AngleAxis<double> Axis;
    typedef Eigen::Vector3d Vector;
    Axis roll_rotation(rotations.x() * degree_to_rad, Vector::UnitX());
    Axis pitch_rotation(rotations.y() * degree_to_rad, Vector::UnitY());
    Axis yaw_rotation(rotations.z() * degree_to_rad, Vector::UnitZ());
    return (roll_rotation * pitch_rotation * yaw_rotation).matrix();
}

/**
 * Creates a transformation matrix including a offset and the rotational part
 * \param transform: transform vector
 * \param rotations: rotations for the matrix
 */
static BITBOTS_INLINE Eigen::Matrix4d create_transform(const Eigen::Vector3d& transfrom, const Eigen::Vector3d& rotations)
{
    Eigen::Matrix4d result;
    result.col(3).head<3>() =  transfrom;
    result.row(3) << 0, 0, 0, 1;
    result.block<3, 3>(0, 0) = create_rotation_matrix(rotations);
    return result;
}

/**
 * Creates a offsetvector for prepared mass vectors in a given container
 * \param offsets: the prepared mass offsets
 */
static BITBOTS_INLINE Eigen::Vector4d mass_from_offsets(const Eigen::Matrix4Xd& offsets) {
    Eigen::Vector4d offset = offsets.rowwise().sum();
    if(offset(3) == 0) {
        return Eigen::Vector4d::Zero();
    }
    offset.head<3>() /= offset(3);

    return offset;
}

/**
 * Creates a massvector representing the centre of gravity from a given list of mass vectors
 * \param masses: The mass vectors, that will be combined
 */
static BITBOTS_INLINE Eigen::Vector4d create_mass_offset(const Eigen::Matrix4Xd& masses) {
    Eigen::Matrix4Xd offsets(4,masses.cols());
    for(uint i = 0; i < (uint)masses.cols(); ++i) {
        offsets.col(i) << masses(3, i) * masses.col(i).head<3>(), masses(3, i);
    }
    return mass_from_offsets(offsets);
}

static BITBOTS_INLINE Eigen::Vector4d create_mass_offset(const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >& masses) {
    Eigen::Matrix4Xd mat(4, masses.size());
    for(uint i = 0; i < masses.size(); ++i) {
        mat.col(i) = masses[i];
    }
    return create_mass_offset(mat);
}

/**
 * Computes the inverse of a chain matrix
 * \param chain_matrix: The chain matrix to be inverted
 */
// static BITBOTS_INLINE Eigen::Matrix4d inverse_chain_matrix(const Eigen::Matrix4d chain_matrix) {
//     Eigen::Matrix4d inverse;
//     inverse.block<3, 3>(0, 0) = chain_matrix.block<3, 3>(0, 0).inverse();
//     inverse.col(3).head<3>() = -(inverse.block<3, 3>(0, 0) * chain_matrix.col(3).head<3>());
//     inverse.row(3) = Eigen::RowVector4d(0, 0, 0, 1);
//     return inverse;
// }
static BITBOTS_INLINE Eigen::Affine3d inverse_chain_matrix(const Eigen::Affine3d& chain_matrix) {
    return chain_matrix.inverse();
}

template<bool allow_jump_to_opposite>
BITBOTS_INLINE void KJoint::check_angle() {
    if(m_angle < -pi)m_angle+=2*pi;
    if(m_angle > pi)m_angle-=2*pi;
    if(m_angle > m_max_angle) {
        double d = m_angle - m_max_angle;
        if(allow_jump_to_opposite && d > 0.2)
            m_angle = m_min_angle;
        else
            m_angle = fmin(m_angle, m_max_angle);
    }
    if(m_angle < m_min_angle) {
        double d = m_min_angle - m_angle;
        if(allow_jump_to_opposite && d > 0.2)
            m_angle = m_max_angle;
        else
            m_angle = fmax(m_angle, m_min_angle);
    }
}

KJoint::KJoint()
: KJoint(-1)
{}

KJoint::KJoint(int motor_id)
:m_transform(Matrix::Identity()), m_inverse_transform(Matrix::Identity()),m_chain_matrix(Matrix::Identity()),
m_mass_offset(Vector::Zero()),m_mass_endpoint(Vector::Zero()),m_chain_masspoint(Vector::Zero()), m_angle(0), m_min_angle(0),m_max_angle(0), m_motor_id(motor_id)
{}

KJoint::KJoint(const Vector3& transform_from_parent, const Vector3& rotations, const Masses& masses, const Vector3& angles, int motor_id)
:m_transform(create_transform(transform_from_parent, rotations)), m_inverse_transform(Matrix::Identity()), m_chain_matrix(create_transform(transform_from_parent, rotations)),
m_mass_offset(create_mass_offset(masses)),m_mass_endpoint(create_mass_offset(masses)),m_chain_masspoint(create_mass_offset(masses)),
m_angle(angles(0) * degree_to_rad), m_min_angle(angles(1) * degree_to_rad), m_max_angle(angles(2) * degree_to_rad), m_motor_id(motor_id)
{
    check_angle();
}

KJoint::KJoint(const KJoint& other)
:m_transform(other.m_transform), m_inverse_transform(other.m_inverse_transform), m_chain_matrix(other.m_chain_matrix), m_mass_offset(other.m_mass_offset),
m_mass_endpoint(other.m_mass_endpoint), m_chain_masspoint(other.m_chain_masspoint), m_angle(other.m_angle), m_min_angle(other.m_min_angle), m_max_angle(other.m_max_angle),
m_motor_id(other.m_motor_id)
{
    check_angle();
}

template<KJoint::AngleCheck checked>
void KJoint::update_angle(const double degree, const AngleCheck p_checked __attribute__((unused)) ) {
    m_angle = degree * degree_to_rad;
    if(checked) {
        check_angle();
    }
}

template<KJoint::AngleCheck checked>
void KJoint::update_angle_rad(const double radians, const AngleCheck p_checked __attribute__((unused)) ) {
    m_angle = radians;
    if(checked) {
        check_angle();
    }
}

bool KJoint::all_data_valid() const {
    #define hasNaN(EIGEN_TRANSFORM) EIGEN_TRANSFORM.matrix().hasNaN()
    return !(std::isnan(m_angle) || std::isnan(m_max_angle) || std::isnan(m_min_angle) || hasNaN(m_chain_masspoint) || hasNaN(m_chain_matrix) || hasNaN(m_transform)
            || hasNaN(m_inverse_transform) || hasNaN(m_mass_offset) || hasNaN(m_mass_endpoint));
    #undef hasNaN
}

/*
* Takes a global endpoint vector and calculates the local gradient belonging to joint manipulation with given endpoint
*/
BITBOTS_INLINE const Eigen::Vector3d KJoint::get_local_gradient(const Vector3& endpoint, const AxisType axis) const {
    if(is_static_joint()) {
        return Vector3::Zero();
    }
    Vector3 delta;
    if(axis == AxisType::Position) {
        delta = (endpoint - get_endpoint<3>(axis));
    } else {
        delta = (endpoint - get_endpoint<3>(axis)).normalized();
        if(std::isnan(delta(0))) return Vector3::Zero();
    }
    //Transforming the endpoint from global to the local coordinate system, then perform cross product in local system
    const Vector3 local_delta = (m_chain_matrix.linear().inverse() * delta);
    return Vector3::UnitY().cross(local_delta);
}

BITBOTS_INLINE const Eigen::Vector3d KJoint::get_local_cog_gradient() const {
    if(is_static_joint()) {
        return Vector3::Zero();
    }
    //Create the massvector as a forcevector
    Vector3 rest_chain_cog(m_chain_masspoint.head<3>() * m_chain_masspoint(3));

    //Transforming the endpoint from global to the local coordinate system, then perform cross product in local system
    const Vector3 local_delta(m_chain_matrix.linear().inverse() * rest_chain_cog);
    return Vector3::UnitY().cross(local_delta);
}

Eigen::Affine3d KJoint::get_chain_matrix_inverse() const {
    return inverse_chain_matrix(m_chain_matrix);
}

void KJoint::print_joint_for_debug() const {
    //std::cout<<"Transform"<<std::endl<<m_transform<<std::endl<<"Mass Offset"<<std::endl<<m_mass_offset<<std::endl
    //<<"Angles"<<std::endl<<m_angle<<" "<<m_min_angle<<" "<<m_max_angle<<std::endl<<"MotorId: "<<m_motor_id<<std::endl;
    std::cout<<"Mass: "<<get_mass()<<std::endl<<"Endpoint, Offset, Endpoint:"<<std::endl<<(Eigen::Matrix<double,4 ,3>()<<m_mass_endpoint, m_mass_offset, get_endpoint()).finished()<<std::endl;
}

void KJoint::create_inverse_transform_with_follower(const KJoint& next) {
    Matrix3 inverse_rotation = next.m_transform.linear().inverse();
    m_inverse_transform.matrix() << inverse_rotation, - inverse_rotation * next.m_transform.translation().head<3>(),
                            0, 0, 0, 1;
}

BITBOTS_INLINE Eigen::Matrix4d transform_inline_multiplication(const Eigen::Affine3d& transform, double angle) {
    return (Eigen::Matrix4d()<<transform.linear() * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()), transform.translation(),0,0,0,1).finished();
}

#define get_right_transform(inverse, next) \
    (inverse? transform_inline_multiplication(next.m_inverse_transform, -next.m_angle): transform_inline_multiplication(next.m_transform, next.m_angle))

template<bool add_existing, KJoint::InvType inverse>
BITBOTS_INLINE void KJoint::update_chain_masspoint_with_follower(const KJoint& next) {
    // I'm sorry for this complex confusing inline mass vector multiplication
    Vector next_offset((Vector()<<(get_right_transform(inverse, next) * (Vector()<<next.m_chain_masspoint.head<3>(), 1).finished()).head<3>(), next.m_chain_masspoint(3)).finished());
    //Vector own_offset = (Vector() << m_mass_offset.head<3>() * m_mass_offset(3), m_mass_offset(3)).finished();
    // When we consider the existing chain masspoint, we need 3 cols, 2 otherwise
    Eigen::Matrix<double, 4, 2> offsets;
    if(add_existing)
        offsets<<next_offset, m_chain_masspoint;
    else
        offsets<<next_offset, m_mass_offset;
    m_chain_masspoint = create_mass_offset(offsets);
}

#undef transform_inline_multiplication
#undef get_right_transform

template<KJoint::AngleCheck checked, KJoint::InvType inverse>
BITBOTS_INLINE const Eigen::Affine3d& KJoint::update_chain_matrix(const Eigen::Affine3d& chain_matrix, const AngleCheck p_checked __attribute__((unused))) {
    if(checked) {
        check_angle();
    }
    Axis rot((inverse? -m_angle: m_angle), Vector3::UnitY());
    m_chain_matrix.operator=(chain_matrix * (inverse? m_inverse_transform: m_transform));
    //Insert the moved offset and led the mass untouched
    m_mass_endpoint<<(m_chain_matrix * (Vector()<<m_mass_offset.head<3>(), 1).finished()).head<3>(), m_mass_offset(3);
    // Finally apply the transformation after masspoint update
    m_chain_matrix.linear() = m_chain_matrix.linear() * rot;
    return m_chain_matrix;
}

#define ANGLE_CHECK_NONE_TEMPLATE_SPECIALIZATION(RETURN_TYPE, METHOD, TYPE_1, PARAM_1, TYPE_2, PARAM_2) \
template<> \
RETURN_TYPE KJoint::METHOD<KJoint::AngleCheck::Undef>(TYPE_1 PARAM_1, TYPE_2 PARAM_2) { \
    switch(p_checked) { \
        case(KJoint::AngleCheck::unchecked): { \
            return METHOD<KJoint::AngleCheck::unchecked>(PARAM_1, PARAM_2); \
        } \
        default: { \
            return METHOD<KJoint::AngleCheck::check>(PARAM_1, PARAM_2); \
        } \
    } \
}

ANGLE_CHECK_NONE_TEMPLATE_SPECIALIZATION(const Eigen::Affine3d&, update_chain_matrix, const Eigen::Affine3d&, chain_matrix, const AngleCheck, p_checked)
ANGLE_CHECK_NONE_TEMPLATE_SPECIALIZATION(void, update_angle, double, degree, const AngleCheck, p_checked)
ANGLE_CHECK_NONE_TEMPLATE_SPECIALIZATION(void, update_angle_rad, double, radians, const AngleCheck, p_checked)

} } //namespaces
