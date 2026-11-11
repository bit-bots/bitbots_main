#include "hardware/joint_state.h"
#include <gtest/gtest.h>
#include <memory>

namespace livelybot_serial
{
namespace
{

class JointStateTest : public ::testing::Test
{
  protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        node = std::make_shared<rclcpp::Node>("test_joint_state");
        node->declare_parameter("robot.control_type", 0);
        yaw = make_motor(1, "head_yaw_joint");
        pitch = make_motor(2, "head_pitch_joint");
        motors = {yaw.get(), pitch.get()};
    }

    std::unique_ptr<motor> make_motor(int index, const std::string &name)
    {
        const std::string base = "robot.CANboard.No_1_CANboard.CANport.CANport_1.motor.motor" + std::to_string(index);
        node->declare_parameter(base + ".name", name);
        node->declare_parameter(base + ".id", index);
        node->declare_parameter(base + ".type", "General");
        node->declare_parameter(base + ".num", index);
        node->declare_parameter(base + ".pos_limit_enable", false);
        node->declare_parameter(base + ".pos_upper", 0.0);
        node->declare_parameter(base + ".pos_lower", 0.0);
        node->declare_parameter(base + ".tor_limit_enable", false);
        node->declare_parameter(base + ".tor_upper", 0.0);
        node->declare_parameter(base + ".tor_lower", 0.0);
        node->declare_parameter(base + ".controller_defaults.kp", 0.01);
        node->declare_parameter(base + ".controller_defaults.kd", 0.01);
        return std::make_unique<motor>(index, 1, 1, &tx, 2, node);
    }

    rclcpp::Node::SharedPtr node;
    cdc_tr_message_s tx{};
    std::unique_ptr<motor> yaw;
    std::unique_ptr<motor> pitch;
    std::vector<motor *> motors;
};

TEST_F(JointStateTest, WaitsForEveryMotorRegardlessOfFeedbackOrder)
{
    EXPECT_FALSE(make_joint_state(motors));

    pitch->fresh_data(0, 0, 1000, 2000, 0);
    EXPECT_FALSE(make_joint_state(motors));

    yaw->fresh_data(0, 0, 0, 0, 0);
    const auto js = make_joint_state(motors);
    ASSERT_TRUE(js);
    EXPECT_EQ(js->name, (std::vector<std::string>{"head_yaw_joint", "head_pitch_joint"}));
    EXPECT_DOUBLE_EQ(js->position[0], 0.0);
    EXPECT_DOUBLE_EQ(js->position[1], pitch->get_current_motor_state()->position);
    EXPECT_DOUBLE_EQ(js->velocity[1], pitch->get_current_motor_state()->velocity);
    EXPECT_EQ(js->effort.size(), motors.size());
}

TEST_F(JointStateTest, DoesNotPublishPartialStateWhenLastMotorIsUnknown)
{
    yaw->fresh_data(0, 0, 1000, 0, 0);
    EXPECT_FALSE(make_joint_state(motors));
}

TEST_F(JointStateTest, UsesLatestFeedbackAfterInitialization)
{
    yaw->fresh_data(0, 0, 0, 0, 0);
    pitch->fresh_data(0, 0, 0, 0, 0);
    ASSERT_TRUE(make_joint_state(motors));

    yaw->fresh_data(0, 0, -1000, -2000, 0);
    const auto js = make_joint_state(motors);
    ASSERT_TRUE(js);
    EXPECT_LT(js->position[0], 0.0);
    EXPECT_LT(js->velocity[0], 0.0);
    EXPECT_DOUBLE_EQ(js->position[0], yaw->get_current_motor_state()->position);
    EXPECT_DOUBLE_EQ(js->velocity[0], yaw->get_current_motor_state()->velocity);
}

TEST_F(JointStateTest, DoesNotPublishEmptyState) { EXPECT_FALSE(make_joint_state({})); }

} // namespace
} // namespace livelybot_serial
