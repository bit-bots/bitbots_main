#include <iostream>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "../kinematic_task.hpp"
#include "robot_dummy.hpp"

using namespace std;
using namespace Eigen;
using namespace ::Robot::Kinematics;
using namespace ::Robot::Kinematics::Test;
namespace Kin = ::Robot::Kinematics;

namespace Test {

#define ID(X) int_to_JointID(X)

TEST(KinematicTask, CreateSimpleChain) {
    KRobot robot = create_complex_robot();
    KinematicTask kt(robot);

    KinematicTask::ChainHolder holder = kt.create_chain(ID(0), ID(2));
    EXPECT_EQ(*holder, robot.get_chain_by_id(0));

    holder = kt.create_chain(ID(0), ID(4));
    EXPECT_EQ(*holder, robot.get_chain_by_id(1));

    holder = kt.create_chain(ID(0), ID(6));
    EXPECT_EQ(*holder, robot.get_chain_by_id(2));
}

TEST(KinematicTask, CreateParticularChain) {
    KRobot robot = create_complex_robot();
    KinematicTask kt(robot);

    KinematicTask::ChainHolder holder = kt.create_chain(ID(1), ID(2));
    EXPECT_EQ(holder->size(), 2);
    EXPECT_EQ(holder.get()[holder->size() - 1]->get_id(), 2);

    KinematicTask task(robot);
    holder = task.create_chain(ID(3), ID(4));
    EXPECT_EQ(holder->size(), 2);
    EXPECT_EQ(holder.get()[holder->size() - 1]->get_id(), 4);

    holder = task.create_chain(ID(5), ID(6));
    EXPECT_EQ(holder->size(), 2);
    EXPECT_EQ(holder.get()[holder->size() - 1]->get_id(), 6);
}

TEST(KinematicTask, CompleteInverseChain) {
    KRobot robot = create_complex_robot();
    KinematicTask kt(robot);

    KinematicTask::ChainHolder holder = kt.create_chain(ID(2), ID(0));
    KRobot::Chain* ref_chain =  &robot.get_chain_by_id(0);
    ASSERT_EQ(holder->size(), ref_chain->size());
    for(unsigned i = 0, j = holder->size() - 1; i < j; ++i, --j) {
        EXPECT_EQ(holder.get()[i]->get_id(), (*ref_chain)[j]->get_id());
    }

    holder = kt.create_chain(ID(4), ID(0));
    ref_chain =  &robot.get_chain_by_id(1);
    ASSERT_EQ(holder->size(), ref_chain->size());
    for(unsigned i = 0, j = holder->size() - 1; i < j; ++i, --j) {
        EXPECT_EQ(holder.get()[i]->get_id(), (*ref_chain)[j]->get_id());
    }

    holder = kt.create_chain(ID(6), ID(0));
    ref_chain =  &robot.get_chain_by_id(2);
    ASSERT_EQ(holder->size(), ref_chain->size());
    for(unsigned i = 0, j = holder->size() - 1; i < j; ++i, --j) {
        EXPECT_EQ(holder.get()[i]->get_id(), (*ref_chain)[j]->get_id());
    }
}

TEST(KinematicTask, ChainOverRoot) {
    KRobot robot = create_complex_robot();
    KinematicTask kt(robot);

    KRobot::Chain* ref1, *ref2;
    KinematicTask::ChainHolder holder = kt.create_chain(ID(2), ID(4));
    ref1 = &robot.get_chain_by_id(0);
    ref2 = &robot.get_chain_by_id(1);
    EXPECT_EQ(holder->size(), ref1->size() + ref2->size() - 1);
    ASSERT_TRUE(holder.get()[0]->get_id() != 0);
    unsigned i = 0;
    while(holder.get()[i]->get_id() != 0) {
        EXPECT_TRUE(holder.get()[i].evaluate_option(KJointChainMember::SingleOptions::is_inverse));
        EXPECT_EQ(holder.get()[i]->get_id() , (*ref1)[ref1->size() - 1 - i]->get_id());
        ++i;
    }
    EXPECT_TRUE(holder.get()[i].evaluate_option(KJointChainMember::SingleOptions::is_inverse));
    EXPECT_EQ(holder.get()[i]->get_id() , (*ref1)[ref1->size() - 1 - i]->get_id());
    unsigned j = 0;
    do {
        ++i;++j;
        EXPECT_FALSE(holder.get()[i].evaluate_option(KJointChainMember::SingleOptions::is_inverse));
        EXPECT_EQ(holder.get()[i]->get_id() , (*ref2)[j]->get_id());
    } while(holder->size() > i + 1);

}

TEST(KinematicTask, CoGChainNoChainDisabled) {
    KRobot robot = create_complex_robot();
    KinematicTask task(robot);

    KinematicTask::ChainHolder holder = task.create_centre_of_gravity_chain(int_to_ChainID(0));
    KRobot::Chain& chain = *holder;
    unsigned chain_size = 0;
    for(const KRobot::Chain& _chain: robot.get_chains()) {
        chain_size += _chain.size();
    }
    EXPECT_EQ(chain.size(), chain_size);
    for(const KJointChainMember& mem: chain) {
        EXPECT_TRUE((bool)mem.evaluate_option(KJointChainMember::Options::need_to_ignore) == (bool)mem->is_static_joint());
    }
    for(unsigned i = 0; chain[i]->get_id() != 0; ++i) {
        EXPECT_TRUE(chain[i].evaluate_option(KJointChainMember::SingleOptions::is_inverse));
    }
}

TEST(KinematicTask, CoGChainInverseChainDisabled) {
    KRobot robot = create_complex_robot();
    KinematicTask task(robot);

    KinematicTask::ChainHolder holder = task.create_centre_of_gravity_chain(int_to_ChainID(0));
    KRobot::Chain& chain = *holder;
    unsigned chain_size = 0;
    for(const KRobot::Chain& _chain: robot.get_chains()) {
        chain_size += _chain.size();
    }
    for(unsigned i = 0; chain[i]->get_id() != 0; ++i) {
        EXPECT_TRUE(chain[i].evaluate_option(KJointChainMember::SingleOptions::is_inactive));
    }
}

TEST(KinematicTask, CoGChainDirectChainDisabled) {
    KRobot robot = create_complex_robot();
    KinematicTask task(robot);

    KinematicTask::ChainHolder holder = task.create_centre_of_gravity_chain(int_to_ChainID(0));
    KRobot::Chain& chain = *holder;
    unsigned chain_size = 0;
    for(const KRobot::Chain& _chain: robot.get_chains()) {
        chain_size += _chain.size();
    }
    unsigned i = 0;
    for(; chain[i]->get_id() != 0; ++i) {}
    for(; i < chain.size(); ++i) {
        EXPECT_TRUE(chain[i].evaluate_option(KJointChainMember::SingleOptions::is_inactive));
    }
}

/*TEST(KinematicTask, ComplexDisableOptions) {
    KRobot robot = create_simple_long_chain_robot();
    KinematicTask kt(robot);

    const KRobot::Chain& ref_chain = robot.get_chain_by_id(ChainIds::LLegChain);
    ASSERT_GE(ref_chain.size(), 6);
    int begin = ref_chain[1]->get_id(),end = ref_chain[5]->get_id();
    unsigned first_task_id = ref_chain[2]->get_id(), second_task_id = ref_chain[3]->get_id(), third_task_id = ref_chain[4]->get_id();
    KinematicTask::TaskIdContainer task_ids({KinematicTask::TaskIdSet({first_task_id}),KinematicTask::TaskIdSet({second_task_id}),KinematicTask::TaskIdSet({third_task_id})});
    Eigen::Matrix<KJoint::AxisType, 3, 1> axis;axis<<KJoint::AxisType::XAxis, KJoint::AxisType::YAxis, KJoint::AxisType::ZAxis;

    KinematicTask::ChainHolder holder = kt.create_chain(robot, int_to_JointID(begin), int_to_JointID(end), axis);
    KRobot::Chain& chain = *holder;

    EXPECT_FALSE(chain[1].evaluate_option(KJointChainMember::SingleOptions::not_for_XAxis));
    EXPECT_TRUE(chain[1].evaluate_option(KJointChainMember::SingleOptions::not_for_YAxis));
    EXPECT_TRUE(chain[1].evaluate_option(KJointChainMember::SingleOptions::not_for_ZAxis));
    EXPECT_FALSE(chain[1].evaluate_option(KJointChainMember::SingleOptions::not_for_Pos));

    EXPECT_TRUE(chain[2].evaluate_option(KJointChainMember::SingleOptions::not_for_XAxis));
    EXPECT_FALSE(chain[2].evaluate_option(KJointChainMember::SingleOptions::not_for_YAxis));
    EXPECT_TRUE(chain[2].evaluate_option(KJointChainMember::SingleOptions::not_for_ZAxis));
    EXPECT_FALSE(chain[2].evaluate_option(KJointChainMember::SingleOptions::not_for_Pos));

    EXPECT_TRUE(chain[3].evaluate_option(KJointChainMember::SingleOptions::not_for_XAxis));
    EXPECT_TRUE(chain[3].evaluate_option(KJointChainMember::SingleOptions::not_for_YAxis));
    EXPECT_FALSE(chain[3].evaluate_option(KJointChainMember::SingleOptions::not_for_ZAxis));
    EXPECT_FALSE(chain[3].evaluate_option(KJointChainMember::SingleOptions::not_for_Pos));

}*/

} //namespace Test

using namespace ::Test;

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
