#ifndef _JOINT_DUMMY_CREATION_HPP__
#define _JOINT_DUMMY_CREATION_HPP__

#include <Eigen/Core>
#include <vector>

#include "../kinematic_joint.hpp"
#include "../chain_member.hpp"

using Robot::Kinematics::KJoint;
typedef typename KJoint::Masses Masses;
namespace Robot {
namespace Kinematics {
namespace Test {

struct JointParameter{
    Eigen::Vector3d transform_from_parent;
    Eigen::Vector3d rotations;
    Eigen::Matrix3d rot_reference;
    Eigen::Vector4d mass_reference;
    Masses masses;
    Eigen::Vector3d angles;
    int motor_id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    JointParameter(Eigen::Vector3d transform_from_parent, Eigen::Vector3d rotations, Eigen::Matrix3d rot_reference,
        Eigen::Vector4d mass_reference, Masses masses, Eigen::Vector3d angles, int motor_id)
    : transform_from_parent(transform_from_parent), rotations(rotations), rot_reference(rot_reference),
      mass_reference(mass_reference), masses(masses), angles(angles), motor_id(motor_id){}

    JointParameter()
    {}
};

JointParameter get_default_joint_parameter() {
    JointParameter p;
    p.transform_from_parent = Eigen::Vector3d(1,2,3);
    p.rotations = Eigen::Vector3d(90,-90,180);
    p.rot_reference = (Eigen::Matrix3d()<<0, 0,-1,
                                          1, 0, 0,
                                          0, -1,0).finished();
    p.mass_reference = Eigen::Vector4d(1,2,3,4);
    p.masses.push_back(p.mass_reference);
    p.angles = Eigen::Vector3d(0, -180, 180);
    p.motor_id = 42;
    return p;
}

KJoint joint_from_parameter(const JointParameter& p) {
    return KJoint(p.transform_from_parent, p.rotations, p.masses, p.angles, p.motor_id);
}

KJointChainMember member_from_parameter(const JointParameter& jp, KJointChainMember::OptionType opt=0) {
    return KJointChainMember(*(new KJoint(joint_from_parameter(jp))), opt);
}

} } }//namespace

#endif //_JOINT_DUMMY_CREATION_HPP__
