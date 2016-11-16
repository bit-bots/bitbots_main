#ifndef _ROBOT_DUMMY_CREATION_HPP__
#define _ROBOT_DUMMY_CREATION_HPP__

#include <Eigen/Core>
#include "../kinematic_robot.hpp"
#include "joint_dummy.hpp"

using Robot::Kinematics::KRobot;
using Robot::Kinematics::KJoint;
typedef typename KRobot::Chain Chain;
typedef typename KRobot::Chains Chains;
typedef typename KRobot::ChainsTemplate ChainsTemplate;
typedef typename KRobot::IdMapping IdMapping;
typedef typename KRobot::ChainMapping ChainMapping;
typedef typename KRobot::Joints Joints;
typedef typename KRobot::JacobiType JacobiType;

namespace Robot {
namespace Kinematics {
namespace Test {

struct RobotParameter{
    IdMapping id_mapping;
    ChainMapping chain_mapping;
    Joints joints;
    ChainsTemplate chain_template;
    int id;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

KRobot robot_from_parameter(RobotParameter& p) {
    return KRobot(p.id_mapping, p.chain_mapping, p.joints, p.chain_template, p.id);
}

RobotParameter get_simple_parameter() {
    RobotParameter p;
    p.id_mapping.insert(std::pair<std::string, int>("Root", 0));
    p.chain_mapping.insert(std::pair<std::string, int>("Dummy", 0));
    p.chain_mapping.insert(std::pair<std::string, int>("Prime", 1));
    p.joints.push_back(KJoint());
    JointParameter jp = get_default_joint_parameter();
    jp.motor_id = 1;
    p.joints.push_back(joint_from_parameter(jp));
    std::vector<int> chain;
    chain.push_back(1);
    p.chain_template.push_back(chain);
    return p;
}

RobotParameter get_advanced_robot_parameter_with_3_joints() {
    using Eigen::Vector3d;
    using std::pair;
    using std::string;
    RobotParameter p = get_simple_parameter();
    JointParameter jp = get_default_joint_parameter();
    p.joints.clear();
    p.joints.push_back(KJoint(0));
    jp.transform_from_parent = Vector3d::Zero();
    jp.rotations = Vector3d::Zero();
    jp.angles = Vector3d(0, -180, 180);
    jp.motor_id = 1;
    KJoint j = joint_from_parameter(jp);
    p.joints.push_back(j);
    jp.transform_from_parent = Vector3d(2, 1, 0);
    jp.rotations = Vector3d::Zero();
    jp.angles = Vector3d::Zero();
    jp.motor_id = 2;
    j = joint_from_parameter(jp);
    p.joints.push_back(j);
    p.chain_template[0].clear();
    p.chain_template[0].push_back(1);
    p.chain_template[0].push_back(2);
    p.id_mapping.insert(pair<string, int>("Erster", 1));
    p.id_mapping.insert(pair<string, int>("Zweiter", 2));
    p.id = 2;
    return p;
}

// Disable the -Wc99-extensions warnings in the following code block.
// I want to use it to initialize the JointParameter structure
#pragma clang diagnostic ignored "-Wc99-extensions"

KRobot create_complex_robot() {
    using namespace Eigen;
    RobotParameter p = get_simple_parameter();
    p.joints.clear();
    // Root Most mass
    JointParameter jp = JointParameter(
        /*.transform_from_parent =*/ Vector3d::Zero(),
        /*.rotations  =*/ Vector3d::Zero(),
        /*.rot_reference =*/ Matrix3d::Zero(),
        /*.mass_reference =*/ Vector4d::Zero(),
        /*.masses =*/ Masses(1,Vector4d(0,0,0,100)),
        /*.angles =*/ Vector3d::Zero(),
        /*.motor_id =*/ 0
                        );
    p.joints.push_back(joint_from_parameter(jp));
    // Chain 1 start
    jp = JointParameter(
        /*.transform_from_parent =*/ Vector3d(10, 0, 0),
        /*.rotations  =*/ Vector3d::Zero(),
        /*.rot_reference =*/ Matrix3d::Zero(),
        /*.mass_reference = */Vector4d::Zero(),
        /*.masses =*/ Masses(1,Vector4d(0,0,0,10)),
        /*.angles =*/ Vector3d(0, -180, 180),
        /*.motor_id =*/ 1
         );
    p.joints.push_back(joint_from_parameter(jp));
    // Chain 1 end
    jp = JointParameter(
        /*.transform_from_parent =*/ Vector3d(10, 0, 0),
        /*.rotations  =*/ Vector3d::Zero(),
        /*.rot_reference =*/ Matrix3d::Zero(),
        /*.mass_reference =*/ Vector4d::Zero(),
        /*.masses =*/ Masses(1,Vector4d(0,0,0,10)),
        /*.angles =*/ Vector3d(0, 0, 0),
        /*.motor_id =*/ 2
    );
    p.joints.push_back(joint_from_parameter(jp));
    // Chain 2 start
    jp = JointParameter(
        /*.transform_from_parent =*/ Vector3d(-10, 0, 0),
        /*.rotations  =*/ Vector3d::Zero(),
        /*.rot_reference =*/ Matrix3d::Zero(),
        /*.mass_reference =*/ Vector4d::Zero(),
        /*.masses =*/ Masses(1,Vector4d(0,0,0,10)),
        /*.angles =*/ Vector3d(0, -180, 180),
        /*.motor_id =*/ 3
         );
    p.joints.push_back(joint_from_parameter(jp));
    // Chain 2 end
    jp = JointParameter(
        /*.transform_from_parent =*/ Vector3d(-10, 0, 0),
        /*.rotations  =*/ Vector3d::Zero(),
        /*.rot_reference =*/ Matrix3d::Zero(),
        /*.mass_reference =*/ Vector4d::Zero(),
        /*.masses =*/ Masses(1,Vector4d(0,0,0,10)),
        /*.angles =*/ Vector3d(0, 0, 0),
        /*.motor_id =*/ 4
    );
    p.joints.push_back(joint_from_parameter(jp));
    // Chain 3 start // inverse
    jp = JointParameter(
        /*.transform_from_parent =*/ Vector3d(0, 0, -10),
        /*.rotations  =*/ Vector3d::Zero(),
        /*.rot_reference =*/ Matrix3d::Zero(),
        /*.mass_reference =*/ Vector4d::Zero(),
        /*.masses =*/ Masses(1,Vector4d(0,0,0,10)),
        /*.angles =*/ Vector3d(0, -180, 180),
        /*.motor_id =*/ 5
         );
    p.joints.push_back(joint_from_parameter(jp));
    // Chain 3 end // inverse
    jp = JointParameter(
        /*.transform_from_parent =*/ Vector3d(0, 0, -10),
        /*.rotations  =*/ Vector3d::Zero(),
        /*.rot_reference =*/ Matrix3d::Zero(),
        /*.mass_reference =*/ Vector4d::Zero(),
        /*.masses =*/ Masses(1,Vector4d(0,0,0,10)),
        /*.angles =*/ Vector3d(0, 0, 0),
        /*.motor_id =*/ 6
         );
    p.joints.push_back(joint_from_parameter(jp));
    std::vector<int> c1, c2, c3;
    c1.push_back(1);c1.push_back(2);
    c2.push_back(3);c2.push_back(4);
    c3.push_back(5);c3.push_back(6);
    p.chain_template.clear();
    p.chain_template.push_back(c1);
    p.chain_template.push_back(c2);
    p.chain_template.push_back(c3);
    p.id = 7;

    return robot_from_parameter(p);
}

KRobot create_simple_long_chain_robot() {
    using namespace Eigen;
    RobotParameter p = get_simple_parameter();
    p.joints.clear();
    // Root Most mass
    JointParameter jp (Vector3d::Zero(), Vector3d::Zero(), Matrix3d::Zero(), Vector4d::Zero(),
                       Masses(1,Vector4d(0,0,0,10)), Vector3d::Zero(), 0);
    p.joints.push_back(joint_from_parameter(jp));
    jp.motor_id = 1;
    p.joints.push_back(joint_from_parameter(jp));
    jp.motor_id = 2;
    p.joints.push_back(joint_from_parameter(jp));
    jp.motor_id = 3;
    p.joints.push_back(joint_from_parameter(jp));
    jp.motor_id = 4;
    p.joints.push_back(joint_from_parameter(jp));
    jp.motor_id = 5;
    p.joints.push_back(joint_from_parameter(jp));
    jp.motor_id = 6;
    p.joints.push_back(joint_from_parameter(jp));
    jp.motor_id = 7;
    p.joints.push_back(joint_from_parameter(jp));
    std::vector<int> c;
    c.push_back(1);c.push_back(2);
    c.push_back(3);c.push_back(4);
    c.push_back(5);c.push_back(6);
    p.chain_template.clear();
    p.chain_template.push_back(c);
    p.chain_template.push_back(c);
    p.chain_template.push_back(c);
    p.chain_template.push_back(c);
    p.chain_template.push_back(c);
    p.id = 7;

    return robot_from_parameter(p);

}

void robot_in_place_adjustment(KRobot& robot, const KJoint& joint) {
    new (&robot[joint.get_id()]) KJoint(joint);
}


} } } //namespace

#endif //_ROBOT_DUMMY_CREATION_HPP__
