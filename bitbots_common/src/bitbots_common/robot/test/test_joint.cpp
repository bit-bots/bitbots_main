#include <iostream>
#include <Eigen/Core>
#include <gtest/gtest.h>

#include "../kinematic_joint.hpp"
#define INCLUDE_JOINT_IMPL_FOR_TESTS
#include "../kinematic_joint-impl.hpp"
#undef INCLUDE_JOINT_IMPL_FOR_TESTS
#include "joint_dummy.hpp"

using namespace Eigen;
using namespace ::Robot::Kinematics;
using namespace ::Robot::Kinematics::Test;
namespace Kin = ::Robot::Kinematics;
using namespace std;

namespace Test {

typedef KJoint::AxisType AxisType;
typedef KJoint::AngleCheck AngleCheck;
typedef KJoint::Masses Masses;

static const Matrix4d id4 = Matrix4d::Identity();
static const Affine3d idAffine3d = Affine3d::Identity();

#define ASSERT_EQ_(expected, actual) ASSERT_EQ(expected, actual)<< "But was:"<<endl<< actual<< endl<<"instead of"<<endl<<expected
#define EXPECT_EQ_(expected, actual) EXPECT_EQ(expected, actual)<< "But was:"<<endl<< actual<< endl<<"instead of"<<endl<<expected

TEST(Joint, defaultJointIsStatic) {
    KJoint j;
    EXPECT_TRUE(j.is_static_joint());
}

TEST(Joint, defaultJointIsSetToZeroValues) {
    KJoint j;
    EXPECT_EQ(j.get_angle(), 0);
    EXPECT_EQ_(j.get_chain_matrix().matrix(), Matrix4d::Identity());
    EXPECT_EQ_(j.get_transform().matrix(), Matrix4d::Identity());
}

TEST(Joint, defaultJointHasSimpleMembers) {
    KJoint j;
    EXPECT_EQ_(j.get_transform().matrix(), Matrix4d::Identity());
}

TEST(Joint, ChainMatrixIsInitedWithTransform) {
    JointParameter p = get_default_joint_parameter();
    p.transform_from_parent = Vector3d(1,2,3);
    p.rotations = Vector3d::Zero();
    KJoint j = joint_from_parameter(p);
    EXPECT_EQ_(j.get_chain_matrix().matrix(), j.get_transform().matrix());
    EXPECT_EQ_(j.get_chain_matrix().matrix(), (Matrix4d()<<Vector4d(1,0,0,0),Vector4d(0,1,0,0),Vector4d(0,0,1,0),Vector4d(1,2,3,1)).finished());
}

TEST(Joint, jointWillBeCreatedCorrect) {
    JointParameter p = get_default_joint_parameter();
    KJoint j = joint_from_parameter(p);
    EXPECT_EQ_(j.get_mass(), p.mass_reference(3));
    EXPECT_EQ_(j.get_centre_of_gravity(), p.mass_reference);
    Matrix3d tranform = j.get_transform().linear();
    Array<double, 3, 3> diff = (tranform - p.rot_reference).array().abs();
    EXPECT_LT(diff.abs().maxCoeff(), 1e-10);
    EXPECT_EQ_(j.get_angle(), p.angles(0));
    Vector3d joint_pos(j.get_transform().translation().head<3>());
    EXPECT_EQ_(joint_pos, p.transform_from_parent);
    //BIT_BOTS_ASSERT(j.get_endpoint<3>(), joint_pos, "");//Endpoint ist chain_matrix
    EXPECT_EQ_(j.get_id(), p.motor_id);
}

TEST(Joint, centreOfGravityIsCorrect) {
    JointParameter p = get_default_joint_parameter();
    p.masses.clear();
    Vector4d m1(1,2,3,4), m2(-1,-2,-3,4),m(0,0,0,8);
    p.masses.push_back(m1);
    p.masses.push_back(m2);
    KJoint j = joint_from_parameter(p);
    // Zwei Massen löschen sich aus
    EXPECT_EQ_(j.get_centre_of_gravity(), m);

    p.masses.clear();
    Vector4d n1(1,2,0,4), n2(0,0,0,4),n(0.5,1,0,8);
    p.masses.push_back(n1);
    p.masses.push_back(n2);
    KJoint k = joint_from_parameter(p);
    // Zwei Massen löschen sich aus
    EXPECT_EQ_(k.get_centre_of_gravity(), n);
}

TEST(Joint, centreOfGravityFromChainIsCorrect) {
    JointParameter p = get_default_joint_parameter();
    p.masses.clear();
    p.rotations = Vector3d::Zero();
    p.transform_from_parent = Vector3d::Zero();
    Vector4d m1(1,2,3,4), m2(-1,-2,-3,4),m(0,0,0,8);
    p.masses.push_back(m1);
    p.masses.push_back(m2);
    KJoint j = joint_from_parameter(p);
    // Zwei Massen löschen sich aus
    EXPECT_EQ_(j.get_centre_of_gravity(), m);

    p.masses.clear();
    p.transform_from_parent = Vector3d(-0.5, -1, 0);
    Vector4d n1(1,2,0,4), n2(0,0,0,4),n(0,0,0,8);
    p.masses.push_back(n1);
    p.masses.push_back(n2);
    KJoint k = joint_from_parameter(p);
    k.update_chain_matrix(j.get_chain_matrix(), KJoint::AngleCheck::check);
    // Zwei Massen löschen sich aus
    EXPECT_EQ_(k.get_centre_of_gravity(), n);
}

TEST(Joint, chainMatrix) {
    JointParameter p = get_default_joint_parameter();
    KJoint j = joint_from_parameter(p);
    Affine3d ref;
    j.update_chain_matrix(idAffine3d, AngleCheck::unchecked);
    ref.matrix() << p.rot_reference, p.transform_from_parent,
                    0,0,0,1;
    Array<double, 4, 4> diff = (j.get_chain_matrix().matrix() - ref.matrix()).matrix().array().abs();
    EXPECT_LT(diff.abs().maxCoeff(), 1e-10);
    j.update_chain_matrix(ref, AngleCheck::unchecked);
    diff = (j.get_chain_matrix().matrix() - (ref * ref).matrix()).array().abs();
    EXPECT_LT(diff.abs().maxCoeff(), 1e-10);
}

TEST(Joint, jointIsStaticDependingOnAngles) {
    JointParameter p = get_default_joint_parameter();
    p.angles = Vector3d(0,0,0);
    KJoint j = joint_from_parameter(p);
    EXPECT_TRUE(j.is_static_joint());
    p.angles = Vector3d(0,1,0);
    KJoint k = joint_from_parameter(p);
    EXPECT_FALSE(k.is_static_joint());
    p.angles = Vector3d(0,0,1);
    KJoint l = joint_from_parameter(p);
    EXPECT_FALSE(l.is_static_joint());
}

TEST(Joint, localGradient) {
    JointParameter p = get_default_joint_parameter();
    p.rotations = Vector3d(-90,0,0);
    p.transform_from_parent = Vector3d(0,0,0);
    KJoint j = joint_from_parameter(p);
    Vector3d dist_v = Vector3d(3, 0, 0);
    Vector3d expected = Vector3d(0, 0, -3);
    EXPECT_EQ_(j.get_local_gradient(dist_v, AxisType::Position), expected);

    dist_v = Vector3d(0, 2, 0);
    expected = Vector3d(2, 0, 0);
    EXPECT_EQ_(j.get_local_gradient(dist_v, AxisType::Position), expected);
    dist_v = Vector3d(0, 0, 2);
    expected = Vector3d::Zero();
    EXPECT_LT((j.get_local_gradient(dist_v, AxisType::Position) - expected).array().abs().maxCoeff(), 1e-15);
}

TEST(Joint, localGradientWithAxis) {
    JointParameter p = get_default_joint_parameter();
    p.rotations = Vector3d(-90,0,0);
    p.transform_from_parent = Vector3d(0,0,0);
    KJoint j = joint_from_parameter(p);
    Vector3d expected = Vector3d::Zero();
    Vector3d dist_v = Vector3d(1, 0, 0);
    EXPECT_EQ_(j.get_local_gradient(dist_v, AxisType::XAxis), expected);

    dist_v = Vector3d(1, 0, 0);
    expected = Vector3d(0, 0, -sqrt(0.5));
    Vector3d result = j.get_local_gradient(dist_v, AxisType::YAxis);
    Array3d diff = (result - expected).array().abs();
    EXPECT_LT(diff.abs().maxCoeff(), 1e-10);
    dist_v = Vector3d(1, 0, 0);
    expected = Vector3d(-sqrt(0.5), 0, -sqrt(0.5));
    result = j.get_local_gradient(dist_v, AxisType::ZAxis);
    diff = (result - expected).array().abs();
    EXPECT_LT(diff.abs().maxCoeff(), 1e-10);
}

TEST(Joint, inverseTransform) {
    JointParameter p = get_default_joint_parameter();
    p.transform_from_parent = Vector3d(2,6,3);
    p.rotations = Vector3d(65,162,234);
    KJoint j = joint_from_parameter(p);
    j.create_inverse_transform_with_follower(j);
    Affine3d transform_and_back = j.get_transform() * j.get_inverse_transform();
    Array<double, 4, 4> diff = (transform_and_back.matrix() - id4).array().abs();
    EXPECT_LT(diff.abs().maxCoeff(), 1e-10);
}

TEST(Joint, inverseChainMatrix) {
    JointParameter p = get_default_joint_parameter();
    p.rotations = Vector3d::Random();
    p.transform_from_parent = Vector3d::Random();
    KJoint j = joint_from_parameter(p), jj = joint_from_parameter(p);
    j.update_chain_matrix(idAffine3d, AngleCheck::unchecked);
    jj.update_chain_matrix(j.get_chain_matrix(), AngleCheck::unchecked);
    Matrix4d inverse = jj.get_chain_matrix_inverse().matrix();
    Matrix4d result = jj.get_chain_matrix().matrix() * inverse;
    Array<double, 4, 4> diff = (id4 - result).array().abs();
    EXPECT_LT(diff.abs().maxCoeff(), 1e-10);
}

TEST(Joint, angleCheck) {
    JointParameter p = get_default_joint_parameter();
    p.angles = Vector3d(100,-90,90);
    KJoint j = joint_from_parameter(p);
    EXPECT_EQ(j.get_angle(), pi / 2);
    j.update_angle(-100,AngleCheck::check);
    EXPECT_EQ(j.get_angle(), -pi / 2);
    j.update_angle(-100, AngleCheck::unchecked);
    EXPECT_EQ(j.get_angle(), - 100 * degree_to_rad);
}

TEST(Joint, getLocalCogGradient) {
    JointParameter p = get_default_joint_parameter();
    p.masses.clear();
    p.rotations = Vector3d::Zero();
    Vector4d mass(0, 0, 0, 5);
    Vector4d mass_2(2, 0, 0, 5);
    Vector3d expected(0, 0, -10);
    p.masses.push_back(mass);
    p.transform_from_parent = Vector3d(2000, 1000, 0);
    KJoint j = joint_from_parameter(p);
    EXPECT_EQ_(j.get_local_cog_gradient(), Vector3d::Zero());
    p.masses[0] = mass_2;
    j = joint_from_parameter(p);
    EXPECT_EQ_(j.get_local_cog_gradient(), expected);

    Vector4d mass_3(1, 3, 1, 10);
    Vector3d exp(10, 0, -10);
    p.masses[0] = mass_3;
    j = joint_from_parameter(p);
    EXPECT_EQ_(j.get_local_cog_gradient(), exp);
}

TEST(Joint, updateChainMasspointWithFollower) {
    JointParameter p = get_default_joint_parameter();
    p.masses.clear();
    p.rotations = Vector3d::Zero();
    Vector4d mass(0, 0, 0, 5);
    Vector4d expected_masspoint(1, 0.5, 0, 10);
    p.masses.push_back(mass);
    p.transform_from_parent = Vector3d(2, 1, 0);
    KJoint j = joint_from_parameter(p), jj = joint_from_parameter(p);
    j.update_chain_masspoint_with_follower(jj);
    EXPECT_EQ_(j.get_chain_masspoint(), expected_masspoint);
}

TEST(Joint, updateChainMasspointWithFollowerAndOwnOffset) {
    JointParameter p = get_default_joint_parameter();
    p.masses.clear();
    p.rotations = Vector3d::Zero();
    Vector4d mass(10, 0, 0, 5);
    Vector4d expected_masspoint(11, 0.5, 0, 10);
    p.masses.push_back(mass);
    p.transform_from_parent = Vector3d(2, 1, 0);
    KJoint j = joint_from_parameter(p), jj = joint_from_parameter(p);
    j.update_chain_masspoint_with_follower(jj);
    EXPECT_EQ_(j.get_chain_masspoint(), expected_masspoint);
}

} //namespace Test

using namespace ::Test;

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
