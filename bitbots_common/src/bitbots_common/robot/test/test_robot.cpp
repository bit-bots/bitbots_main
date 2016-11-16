#include <iostream>
#include <Eigen/Core>
#include <gtest/gtest.h>

// A define to force the jointids.hpp to propagate the right enum values
//#define USE_DARWIN_JOINTS

#include "../kinematic_joint.hpp"
#include "../kinematic_robot.hpp"

#include "robot_dummy.hpp"
#include "../kinematic_task.hpp"

using namespace Eigen;
using namespace Robot::Kinematics;
using namespace ::Robot::Kinematics::Test;
using namespace std;

namespace Test {

typedef KJoint::AxisType AxisType;
typedef KRobot::MultipleAxisType MultipleAxisType;
typedef KRobot::MultipleErrorType MultipleErrorType;

#define ASSERT_EQ_(expected, actual) ASSERT_EQ(expected, actual)<< "But was:"<<endl<< actual<< endl<<"instead of"<<endl<<expected
#define EXPECT_EQ_(expected, actual) EXPECT_EQ(expected, actual)<< "But was:"<<endl<< actual<< endl<<"instead of"<<endl<<expected

TEST(Robot, defaultRobotIsEmpty) {
    KRobot robot;
    EXPECT_EQ(robot.get_chains().size(), 0);
    EXPECT_EQ(robot.get_max_motor_id(), 0);
    EXPECT_EQ(robot.get_mass(), 0);
}

TEST(Robot, fillJacobiMatrix) {
    RobotParameter p = get_advanced_robot_parameter_with_3_joints();
    JointParameter jp = get_default_joint_parameter();

    KRobot robot = robot_from_parameter(p);
    robot.update(VectorXd::Zero(2, 1));

    Chain c = robot.get_chain_by_id(0);
    JacobiType ja(3,1);

    fill_jacobi_matrix_l(ja, c, AxisType::Position);
    Vector3d ref(0,0,-2);
    EXPECT_EQ_(ja, ref);
}

TEST(Robot, fillJacobiMatrixWithUnmodifiedAxis) {
    RobotParameter p = get_advanced_robot_parameter_with_3_joints();
    JointParameter jp = get_default_joint_parameter();

    KRobot robot = robot_from_parameter(p);
    robot.update(VectorXd::Zero(2, 1));

    Chain c = robot.get_chain_by_id(0);
    JacobiType ja = JacobiType::Zero(3,2);

    Vector3d ref;

    fill_jacobi_matrix_l(ja, c, AxisType::XAxis);
    ref << 0, 0, 0;
    EXPECT_EQ_(ja.rowwise().sum(), ref);

    fill_jacobi_matrix_l(ja, c, AxisType::YAxis);
    ref << 0, 0, 0;
    EXPECT_EQ_(ja.rowwise().sum(), ref);

    fill_jacobi_matrix_l(ja, c, AxisType::ZAxis);
    ref << 0, 0, 0;
    EXPECT_EQ_(ja.rowwise().sum(), ref);
}

TEST(Robot, fillJacobiMatrixWithModifiedAxis) {
    RobotParameter p = get_advanced_robot_parameter_with_3_joints();
    JointParameter jp = get_default_joint_parameter();
    jp.rotations = Vector3d(0, 90, 0);
    jp.transform_from_parent = Vector3d(2, 1, 0);
    p.joints[2] = joint_from_parameter(jp);

    KRobot robot = robot_from_parameter(p);
    robot.update(VectorXd::Zero(2, 1));

    Chain c = robot.get_chain_by_id(0);
    JacobiType ja = JacobiType::Zero(3,2);

    Vector3d ref;

    fill_jacobi_matrix_l(ja, c, AxisType::XAxis);
    ref << -sqrt(0.5), 0, sqrt(0.5);
    Array3d diff = (ja.rowwise().sum() - ref).array().abs();
    ASSERT_LT(diff.maxCoeff(), 1e-10);

    fill_jacobi_matrix_l(ja, c, AxisType::YAxis);
    ref << 0, 0, 0;
    diff = (ja.rowwise().sum() - ref).array().abs();
    ASSERT_LT(diff.maxCoeff(), 1e-20);

    fill_jacobi_matrix_l(ja, c, AxisType::ZAxis);
    ref << -sqrt(0.5), 0, -sqrt(0.5);
    diff = (ja.rowwise().sum() - ref).array().abs();
    ASSERT_LT(diff.maxCoeff(), 1e-14);
}

TEST(Robot, inverseChainWithAxisKT) {
    RobotParameter p = get_advanced_robot_parameter_with_3_joints();
    JointParameter jp = get_default_joint_parameter();
    jp.rotations = Vector3d(0, 90, 0);
    jp.transform_from_parent = Vector3d(2, 1, 0);
    jp.motor_id = 2;
    p.joints[2] = joint_from_parameter(jp);
    KRobot robot = robot_from_parameter(p);
    AxisType axis_t = AxisType::XAxis;
    robot.update(VectorXd::Zero(2, 1));
    Matrix4d chain_matrix = robot.get_joint_by_id(2).get_chain_matrix().matrix();
    Vector3d axis = chain_matrix.col(axis_t).head<3>();
    Array3d diff = (axis - Vector3d(0, 0, -1)).array().abs();
    ASSERT_LT(diff.abs().maxCoeff(), 1e-15);
    KinematicTask kt(robot);
    kt.set_target_values((KRobot::MultipleAxisType(1,1)<<axis_t).finished(),JointIds::Root, int_to_JointID(robot.get_chain(0).back()->get_id()),
                         (KRobot::MultipleTargetType(3,1)<<1,0,0).finished(), (MultipleErrorType(1,1)<<1e-10).finished());
    //robot.inverse_chain(0, Vector3d(1, 0, 0), (MultipleErrorType(1,1)<<1e-10).finished(), 100, (MultipleAxisType(1,1)<< axis_t).finished(), 1e-12);
    robot.inverse_chain(kt, 100, 1);
    chain_matrix = robot.get_joint_by_id(2).get_chain_matrix().matrix();
    axis = chain_matrix.col(axis_t).head<3>();
    //There is alway some uncertainty in the results:
    if(fabs(axis(2)) < 1e-10) axis(2) = 0;
    diff = (axis - Vector3d(1, 0, 0)).array().abs();
    ASSERT_LT(diff.abs().maxCoeff(), 1e-22);
}

TEST(Robot, inverseChainWithPositionKT) {
    RobotParameter p = get_advanced_robot_parameter_with_3_joints();
    JointParameter jp = get_default_joint_parameter();
    jp.rotations = Vector3d(0, 90, 0);
    jp.angles = Vector3d::Zero();
    jp.transform_from_parent = Vector3d(20, 10, 0);
    jp.motor_id = 2;
    p.joints[2] = joint_from_parameter(jp);
    KRobot robot = robot_from_parameter(p);

    AxisType axis_t = AxisType::Position;
    robot.update(VectorXd::Zero(2, 1));
    Matrix4d chain_matrix = robot.get_joint_by_id(2).get_chain_matrix().matrix();
    Vector3d axis = chain_matrix.col(axis_t).head<3>();
    EXPECT_EQ_(axis, Vector3d(20, 10, 0));
    Vector3d target(18.0429, 10, 8.62868);

    KinematicTask kt(robot);
    kt.set_target_values((KRobot::MultipleAxisType(1,1)<<axis_t).finished(),JointIds::Root, int_to_JointID(robot.get_chain(0).back()->get_id()),
                         target, (MultipleErrorType(1,1)<<1e-4).finished());
    robot.inverse_chain(kt, 100, 1);
    chain_matrix = robot.get_joint_by_id(2).get_chain_matrix().matrix();
    axis = chain_matrix.col(axis_t).head<3>();
    //There is alway some uncertainty in the results:
    Array3d diff = (axis - target).array().abs();
    ASSERT_LT(diff.abs().maxCoeff(), 0.01);
}


TEST(Robot, dynamicChain) {
    RobotParameter p = get_advanced_robot_parameter_with_3_joints();
    JointParameter jp = get_default_joint_parameter();
    jp.motor_id = 0;
    p.id = 2;
    p.joints.clear();
    p.joints.push_back(joint_from_parameter(jp));
    jp.angles = Vector3d(0, 0, 90);
    jp.motor_id = 1;
    p.joints.push_back(joint_from_parameter(jp));
    jp.transform_from_parent = Vector3d(1, 0, 0);
    jp.motor_id = 2;
    p.joints.push_back(joint_from_parameter(jp));
    p.chain_template.clear();
    p.chain_template.push_back(vector<int>());
    p.chain_template[0].push_back(1);
    p.chain_template[0].push_back(2);

    KRobot robot = robot_from_parameter(p);
    robot.update(VectorXd::Zero(2,1));

    KRobot::Chain c = robot.get_chain_by_id(0);
    JacobiType j = JacobiType::Zero(3, 2);
    fill_jacobi_matrix_l(j, c, AxisType::Position);
    ASSERT_FALSE((j == JacobiType::Zero(3, 2)));
    c[1].set_option(KJointChainMember::SingleOptions::not_for_Pos);
    fill_jacobi_matrix_l(j, c, AxisType::Position);
    //EXPECT_EQ_(j, JacobiType::Zero(3, 2));
    EXPECT_LT(j.array().abs().sum(), 1e-20);
}

/**
 * The following test create a simple inverse chain, update it manually somehow and perform a filling of the jacobian matrix.
 */
TEST(Robot, jacobiFromInverseChain) {
    typedef KJointChainMember::SingleOptions SO;
    KRobot::Chain chain;
    JointParameter jp = get_default_joint_parameter();
    jp.rotations = Vector3d(0, 0, 0);

    jp.motor_id = 2;
    jp.angles = Vector3d(0, 0, 0);
    jp.transform_from_parent = Vector3d(1, 0, 0);

    chain.push_back(member_from_parameter(jp, SO::is_inactive | SO::is_inverse));

    jp.motor_id = 1;
    jp.angles = Vector3d(0, -180, 180);
    jp.transform_from_parent = Vector3d(1, 0, 0);

    chain.push_back(member_from_parameter(jp, SO::is_inverse));

    jp.angles = Vector3d(0, 0, 0);
    jp.motor_id = 0;
    jp.transform_from_parent = Vector3d(0, 0, 0);

    chain.push_back(member_from_parameter(jp, SO::is_inactive | SO::is_inverse));

    KRobot::JacobiType j(3, 2), j_ref(3, 2);

    chain[1]->create_inverse_transform_with_follower(*chain[0]);
    chain[2]->create_inverse_transform_with_follower(*chain[1]);

    const Affine3d id4 = Affine3d::Identity(), * prev = &id4;
    for(KJointChainMember& mem: chain) {
        prev = &(*mem).update_chain_matrix<KJoint::AngleCheck::check, KJoint::InvType::is_inverse>(*prev);
    }
    j_ref<<Vector3d(0, 0, -1),Vector3d(0, 0, 0);

    fill_jacobi_matrix_l(j, chain);

    //EXPECT_EQ_(j, j_ref);
    EXPECT_LT((j-j_ref).array().abs().sum() , 1e-10);

    //cleanup
    delete chain[0].ptr();
    delete chain[1].ptr();
    delete chain[2].ptr();
}

TEST(Robot, jacobiFromInverseChainWithRotation) {
    typedef KJointChainMember::SingleOptions SO;
    KRobot::Chain chain;
    JointParameter jp = get_default_joint_parameter();

    jp.rotations = Vector3d(-90, 0, 0);
    jp.motor_id = 2;
    jp.angles = Vector3d(0, 0, 0);
    jp.transform_from_parent = Vector3d(1, 0, 0);

    chain.push_back(member_from_parameter(jp, SO::is_inactive | SO::is_inverse));

    jp.rotations = Vector3d(0, 0, 0);
    jp.motor_id = 1;
    jp.angles = Vector3d(0, -180, 180);
    jp.transform_from_parent = Vector3d(1, 0, 0);

    chain.push_back(member_from_parameter(jp, SO::is_inverse));

    jp.rotations = Vector3d(0, 0, 0);
    jp.angles = Vector3d(0, 0, 0);
    jp.motor_id = 0;
    jp.transform_from_parent = Vector3d(0, 0, 0);

    chain.push_back(member_from_parameter(jp, SO::is_inactive | SO::is_inverse));

    KRobot::JacobiType j(3, 2), j_ref(3, 2);

    chain[1]->create_inverse_transform_with_follower(*chain[0]);
    chain[2]->create_inverse_transform_with_follower(*chain[1]);

    const Affine3d id4 = Affine3d::Identity(), * prev = &id4;
    for(KJointChainMember& mem: chain) {
        prev = &(*mem).update_chain_matrix<KJoint::AngleCheck::check, KJoint::InvType::is_inverse>(*prev);
    }
    j_ref<<Vector3d(0, 1, 0), Vector3d::Zero();

    fill_jacobi_matrix_l(j, chain);

    ASSERT_LT((j - j_ref).array().abs().sum(), 1e-10);

    //cleanup
    delete chain[0].ptr();
    delete chain[1].ptr();
    delete chain[2].ptr();
}

TEST(Robot, jacobiFromInverseChainWithAngle) {
    typedef KJointChainMember::SingleOptions SO;
    KRobot::Chain chain;
    JointParameter jp = get_default_joint_parameter();

    jp.rotations = Vector3d(0, 0, 0);
    jp.motor_id = 2;
    jp.angles = Vector3d(0, 0, 0);
    jp.transform_from_parent = Vector3d(1, 0, 0);

    chain.push_back(member_from_parameter(jp, SO::is_inactive | SO::is_inverse));

    jp.rotations = Vector3d(0, 0, 0);
    jp.motor_id = 1;
    jp.angles = Vector3d(90, -180, 180);
    jp.transform_from_parent = Vector3d(1, 0, 0);

    chain.push_back(member_from_parameter(jp, SO::is_inverse));

    jp.rotations = Vector3d(0, 0, 0);
    jp.angles = Vector3d(0, 0, 0);
    jp.motor_id = 0;
    jp.transform_from_parent = Vector3d(0, 0, 0);

    chain.push_back(member_from_parameter(jp, SO::is_inactive | SO::is_inverse));

    KRobot::JacobiType j(3, 2), j_ref(3, 2);

    chain[1]->create_inverse_transform_with_follower(*chain[0]);
    chain[2]->create_inverse_transform_with_follower(*chain[1]);

    const Affine3d id4 = Affine3d::Identity(), * prev = &id4;
    for(KJointChainMember& mem: chain) {
        prev = &(*mem).update_chain_matrix<KJoint::AngleCheck::check, KJoint::InvType::is_inverse>(*prev);
    }
    j_ref<<Vector3d(1, 0, 0), Vector3d::Zero();

    fill_jacobi_matrix_l(j, chain);

    EXPECT_EQ_(j, j_ref);
    ASSERT_LT((j - j_ref).array().abs().sum(), 1e-10);

    //cleanup
    delete chain[0].ptr();
    delete chain[1].ptr();
    delete chain[2].ptr();
}

TEST(Robot, SimpleReachableTargetCheck) {
    RobotParameter p = get_simple_parameter();
    p.id = 3;
    p.chain_template = vector<vector<int> >({vector<int>({1,2,3})});
    JointParameter jp = get_default_joint_parameter();
    p.joints.clear();
    jp.angles = Vector3d(0,0, 0);
    jp.transform_from_parent = Vector3d(0,0,0);
    p.joints.push_back(joint_from_parameter(jp));
    jp.motor_id = 1;
    jp.transform_from_parent = Vector3d(10,20,30);
    p.joints.push_back(joint_from_parameter(jp));
    jp.motor_id = 2;
    jp.angles = Vector3d(0,-90, 120);
    p.joints.push_back(joint_from_parameter(jp));
    jp.motor_id = 3;
    jp.angles = Vector3d(0,0, 0);
    p.joints.push_back(joint_from_parameter(jp));

    KRobot robot = robot_from_parameter(p);

    EXPECT_TRUE(KRobot::plausibility_check(robot.get_chain(0), Vector3d(10,20,30)));
    EXPECT_TRUE(KRobot::plausibility_check(robot.get_chain(0), Vector3d(20,40,60)));
    EXPECT_TRUE(KRobot::plausibility_check(robot.get_chain(0), Vector3d(30,60,90)));
    EXPECT_TRUE(KRobot::plausibility_check(robot.get_chain(0), Vector3d(0,0,0)));
    EXPECT_TRUE(KRobot::plausibility_check(robot.get_chain(0), Vector3d(-10,-20,-30)));

    EXPECT_FALSE(KRobot::plausibility_check(robot.get_chain(0), Vector3d(30,60,91)));
    EXPECT_FALSE(KRobot::plausibility_check(robot.get_chain(0), Vector3d(-10,-20,-31)));
}

TEST(Robot, InverseTransformInChain) {
    for(int angle = 0; angle < 360; ++angle) {
        RobotParameter p = get_simple_parameter();
        JointParameter j = get_default_joint_parameter();
        j.motor_id = 0;
        j.transform_from_parent<<0,0,0;
        j.rotations<<0,0,0;
        p.joints[0] = joint_from_parameter(j);
        j.transform_from_parent = Vector3d(1,0,0);
        j.rotations = Vector3d(0,-90,0);
        j.motor_id = 1;
        p.joints[1] = joint_from_parameter(j);
        p.joints[1].update_angle(angle);
        j.rotations = Vector3d(0,0,0);
        j.motor_id = 2;
        p.joints.emplace_back(joint_from_parameter(j));
        p.chain_template[0].push_back(2);
        p.id = 2;

        KRobot robot = robot_from_parameter(p);
        KRobot::Chain inverse_chain;
        for(unsigned i = 2; i < (unsigned) -1 ; --i)
            inverse_chain.emplace_back(robot[i],KJointChainMember::Default|KJointChainMember::is_inverse);

        //Update Robot
        robot.update(Vector2d(angle * degree_to_rad,0));
        // Get endpoint
        Affine3d end_position = robot[2].get_chain_matrix();
        // Reset data
        for(KJointChainMember& mem: inverse_chain) {
            mem->update_chain_matrix(Affine3d(Matrix4d::Zero()));
        }
        // Update again
        KinematicTask::update_robot_chain(robot,inverse_chain,KRobot::both);
        // Get another Position
        Affine3d inverse_position = inverse_chain.back()->get_chain_matrix();

        ASSERT_TRUE((end_position * inverse_position).matrix().isApprox(Matrix4d::Identity()))  <<"End:\n"<<end_position.matrix()
                                                                                                <<"\ninverse\n"<<inverse_position.matrix()<<"\nmul\n"
                                                                                                <<(end_position * inverse_position).matrix();
    }
}

TEST(Robot, InverseTransformInChainWithMass) {
    RobotParameter p = get_simple_parameter();
    JointParameter j = get_default_joint_parameter();
    j.motor_id = 0;
    j.transform_from_parent<<0,0,0;
    j.rotations<<0,0,0;
    j.masses = KJoint::Masses();j.masses.push_back(Vector4d(0,0,0,1));
    p.joints[0] = joint_from_parameter(j);
    j.transform_from_parent = Vector3d(1,0,0);
    j.rotations = Vector3d(0,-90,0);
    j.motor_id = 1;
    j.masses[0] = Vector4d(0,0,0,0);
    p.joints[1] = joint_from_parameter(j);
    p.joints[1].update_angle(90);
    j.rotations = Vector3d(0,0,0);
    j.motor_id = 2;
    j.masses[0] = Vector4d(0,0,0,1);
    p.joints.emplace_back(joint_from_parameter(j));
    p.chain_template[0].push_back(2);
    p.id = 2;

    KRobot robot = robot_from_parameter(p);
    KRobot::Chain inverse_chain;
    for(unsigned i = 2; i < (unsigned) -1 ; --i)
        inverse_chain.emplace_back(robot[i],KJointChainMember::Default|KJointChainMember::is_inverse);

    //Update Robot
    robot.update(Vector2d(90 * degree_to_rad,0));
    KinematicTask::update_robot_chain(robot,robot.get_chain(0),KRobot::both);
    // Get endpoint
    //Vector4d cog = robot.get_centre_of_gravity();
    Vector4d root_cog = robot.get_chain(0).back()->get_centre_of_gravity();
    Vector4d root_cmp = robot.get_chain(0).front()->get_chain_masspoint();
    // Reset data
    for(KJointChainMember& mem: inverse_chain) {
        mem->update_chain_matrix(Affine3d(Matrix4d::Zero()));
    }
    // Update again
    KinematicTask::update_robot_chain(robot,inverse_chain,KRobot::both);
    // Get another Position
    Vector4d end_cog = inverse_chain.back()->get_centre_of_gravity();
    Vector4d end_cmp = inverse_chain.front()->get_chain_masspoint();
    EXPECT_TRUE(((root_cog + end_cog).head<3>().array().abs() < 1e-10).all())<<"Sum\t"<<(root_cog + end_cog).head<3>().transpose()<<"\nRoot\t"<<root_cog.transpose()<<"\nEnd\t"<<end_cog.transpose();
    // The chain setup is symmetric, the both chain masspoints should have different signs
    EXPECT_TRUE(root_cmp.head<3>().isApprox(-end_cmp.head<3>()))<<"Diff\t"<<(root_cmp - end_cmp).transpose()<<"\nRoot\t"<<root_cmp.transpose()<<"\nEnd\t"<<end_cmp.transpose();
    EXPECT_EQ(root_cmp(3), end_cmp(3));
}

#ifndef YAML_NOTFOUND

TEST(Robot,YamlConfig) {
    KRobot* robot = load_robot_from_file();
    EXPECT_TRUE(robot->all_data_valid());
    //robot->update(Eigen::Matrix<double, JointAndChainNumbers::ActiveJoints, 1>::Zero());
    robot->update(Eigen::MatrixXd::Zero(robot->get_max_motor_id(), 1));
    EXPECT_LT((*robot)[JointIds::LFootEndpoint].get_endpoint<3>().z(), 0);
    EXPECT_GT((*robot)[JointIds::LFootEndpoint].get_endpoint<3>().y(), 0);
    EXPECT_LT((*robot)[JointIds::RFootEndpoint].get_endpoint<3>().z(), 0);
    EXPECT_LT((*robot)[JointIds::RFootEndpoint].get_endpoint<3>().y(), 0);
    EXPECT_LT((*robot)[JointIds::RArmEndpoint].get_endpoint<3>().y(), 0);
    EXPECT_GT((*robot)[JointIds::LArmEndpoint].get_endpoint<3>().y(), 0);
    EXPECT_GT((*robot)[JointIds::Camera].get_endpoint<3>().z(), 0);
    delete robot;
}

#endif

} //namespace Test

using namespace ::Test;

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
