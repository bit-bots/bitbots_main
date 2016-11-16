#include <iostream>
#include <Eigen/Core>
#include <gtest/gtest.h>

#include "../joint.hpp"
#include "../pose.hpp"

namespace Test {

TEST(Joint, defaultJointIsInactive) {
    Robot::Joint j;
    ASSERT_FALSE(j.is_active());
    ASSERT_FALSE(j.has_changed());
}

TEST(Joint, setGoalActivatesJoint) {
    Robot::Joint j;
    j.set_goal(1);
    ASSERT_TRUE(j.is_active());
}

TEST(Joint, setPChangesJoint) {
    Robot::Joint j;
    j.set_p(1);
    ASSERT_TRUE(j.has_changed());
}

TEST(Joint, setSpeedChangesJoint) {
    Robot::Joint j;
    j.set_speed(1);
    ASSERT_TRUE(j.has_changed());
}

TEST(Joint, setPositionKeepsJointInactive) {
    Robot::Joint j;
    j.set_position(1);
    ASSERT_FALSE(j.has_changed());
}

TEST(Joint, setGoalRespectsMaxAngles) {
    Robot::Joint j(-2,3);
    EXPECT_TRUE(j.set_goal(1));
    EXPECT_EQ(j.get_goal(), 1);
    EXPECT_TRUE(j.set_goal(2.5));
    EXPECT_EQ(j.get_goal(), 2.5);
    EXPECT_FALSE(j.set_goal(-2.5));
    EXPECT_EQ(j.get_goal(), 2.5);
    EXPECT_FALSE(j.set_goal(4));
    EXPECT_EQ(j.get_goal(), 2.5);
    EXPECT_TRUE(j.set_goal(3));
    EXPECT_EQ(j.get_goal(), 3);
}

TEST(Pose, defaultPoseHasInaktiveJoints) {
    Robot::Pose p;
    for(const Robot::Joint* j_ptr : p.get_const_joints()) {
        EXPECT_FALSE(j_ptr->is_active());
        EXPECT_FALSE(j_ptr->has_changed());
    }
}

TEST(Pose, resetUnchangesJoints) {
    Robot::Pose p;
    for(Robot::Joint* j_ptr : p.get_joints()) {
        j_ptr->set_goal(1);
    }
    p.reset();
    for(Robot::Joint* j_ptr : p.get_joints()) {
        EXPECT_FALSE(j_ptr->has_changed());
    }
}

} //namespace Test
using namespace Test;

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}