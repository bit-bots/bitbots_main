#include <gtest/gtest.h>

#include <bitbots_msgs/msg/joint_command.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <vector>

#include "../src/head_mover.hpp"

using namespace std::chrono_literals;

namespace {

class HeadMoverTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    head_mover_ = std::make_shared<move_head::HeadMover>();
    io_node_ = std::make_shared<rclcpp::Node>("head_mover_test_io");
    joint_state_publisher_ = io_node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
    motor_goal_subscriber_ = io_node_->create_subscription<bitbots_msgs::msg::JointCommand>(
        "head_motor_goals", 10, [this](const bitbots_msgs::msg::JointCommand::SharedPtr) { motor_goals_++; });
    executor_.add_node(head_mover_->get_node());
    executor_.add_node(io_node_);
  }

  void TearDown() override {
    executor_.remove_node(io_node_);
    executor_.remove_node(head_mover_->get_node());
  }

  /// Keeps publishing the joint state while spinning, until the condition holds
  /// or the timeout passes. Publishing repeatedly covers the discovery delay.
  template <typename Predicate>
  bool publishUntil(const sensor_msgs::msg::JointState& joint_state, Predicate condition,
                    std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (condition()) {
        return true;
      }
      joint_state_publisher_->publish(joint_state);
      executor_.spin_some(20ms);
      std::this_thread::sleep_for(10ms);
    }
    return condition();
  }

  static sensor_msgs::msg::JointState makeJointState(const std::vector<std::string>& names) {
    sensor_msgs::msg::JointState joint_state;
    joint_state.name = names;
    joint_state.position.assign(names.size(), 0.1);
    return joint_state;
  }

  std::shared_ptr<move_head::HeadMover> head_mover_;
  rclcpp::Node::SharedPtr io_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Subscription<bitbots_msgs::msg::JointCommand>::SharedPtr motor_goal_subscriber_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  int motor_goals_ = 0;
};

}  // namespace

TEST_F(HeadMoverTest, BuildsTheSearchPatternOnceTheJointStatesBecomeComplete) {
  // The node starts in the look forward search pattern. Its trajectory starts
  // at the current head position, so it cannot be built while the joint states
  // lack a head joint and nothing may be commanded yet.
  const auto incomplete = makeJointState({"head_yaw_joint", "l_knee"});
  publishUntil(incomplete, [] { return false; }, 1s);
  EXPECT_EQ(motor_goals_, 0);

  // Complete joint states arrive without the head mode changing. The build is
  // retried and the pattern starts, rather than the head staying without a
  // trajectory until the next mode change.
  const auto complete = makeJointState({"head_yaw_joint", "head_pitch_joint", "l_knee"});
  EXPECT_TRUE(publishUntil(complete, [this] { return motor_goals_ > 0; }, 5s));
}
