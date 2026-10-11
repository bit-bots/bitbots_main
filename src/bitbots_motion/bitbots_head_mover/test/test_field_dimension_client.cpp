#include <gtest/gtest.h>

#include <bitbots_head_mover/field_dimension_client.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>

using bitbots_head_mover::FieldDimensionClient;
using bitbots_head_mover::FieldDimensions;
using namespace std::chrono_literals;

namespace {

constexpr auto kRetryPeriod = 100ms;

class FieldDimensionClientTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("field_dimension_client_test");
    executor_.add_node(node_);
  }

  void TearDown() override {
    for (const auto& node : extra_nodes_) {
      executor_.remove_node(node);
    }
    executor_.remove_node(node_);
  }

  /// Starts a stand-in for the parameter blackboard under a unique name.
  rclcpp::Node::SharedPtr startBlackboard(const std::string& name) {
    auto blackboard = std::make_shared<rclcpp::Node>(name);
    executor_.add_node(blackboard);
    extra_nodes_.push_back(blackboard);
    return blackboard;
  }

  /// Spins until the condition holds or the timeout passes.
  template <typename Predicate>
  bool spinUntil(Predicate condition, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (condition()) {
        return true;
      }
      executor_.spin_some(10ms);
    }
    return condition();
  }

  void spinFor(std::chrono::milliseconds duration) {
    spinUntil([] { return false; }, duration);
  }

  rclcpp::Node::SharedPtr node_;
  std::vector<rclcpp::Node::SharedPtr> extra_nodes_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

}  // namespace

TEST_F(FieldDimensionClientTest, ReceivesTheDimensionsFromAvailableBlackboard) {
  auto blackboard = startBlackboard("blackboard_available");
  blackboard->declare_parameter("field.size.x", 9.0);
  blackboard->declare_parameter("field.size.y", 6.0);

  std::optional<FieldDimensions> received;
  FieldDimensionClient client(node_, "/blackboard_available", kRetryPeriod,
                              [&received](const FieldDimensions& dimensions) { received = dimensions; });

  ASSERT_TRUE(spinUntil([&] { return received.has_value(); }, 5s));
  EXPECT_TRUE(client.received());
  EXPECT_DOUBLE_EQ(received->length, 9.0);
  EXPECT_DOUBLE_EQ(received->width, 6.0);
}

TEST_F(FieldDimensionClientTest, KeepsRetryingUntilALateBlackboardAnswers) {
  int calls = 0;
  std::optional<FieldDimensions> received;
  FieldDimensionClient client(node_, "/blackboard_late", kRetryPeriod, [&](const FieldDimensions& dimensions) {
    calls++;
    received = dimensions;
  });

  // Several retry periods pass without anyone answering, which used to disable
  // active vision for the rest of the run
  spinFor(5 * kRetryPeriod);
  EXPECT_FALSE(client.received());

  auto blackboard = startBlackboard("blackboard_late");
  blackboard->declare_parameter("field.size.x", 14.0);
  blackboard->declare_parameter("field.size.y", 9.0);

  ASSERT_TRUE(spinUntil([&] { return received.has_value(); }, 5s));
  EXPECT_DOUBLE_EQ(received->length, 14.0);
  EXPECT_DOUBLE_EQ(received->width, 9.0);

  // Once received, the retries stop and the result is reported only once
  spinFor(5 * kRetryPeriod);
  EXPECT_EQ(calls, 1);
}

TEST_F(FieldDimensionClientTest, KeepsWaitingWhileTheDimensionsAreMissing) {
  // The blackboard is up, but does not know the field yet
  auto blackboard = startBlackboard("blackboard_incomplete");

  std::optional<FieldDimensions> received;
  FieldDimensionClient client(node_, "/blackboard_incomplete", kRetryPeriod,
                              [&received](const FieldDimensions& dimensions) { received = dimensions; });
  spinFor(5 * kRetryPeriod);
  EXPECT_FALSE(client.received());

  blackboard->declare_parameter("field.size.x", 9.0);
  blackboard->declare_parameter("field.size.y", 6.0);
  ASSERT_TRUE(spinUntil([&] { return received.has_value(); }, 5s));
  EXPECT_DOUBLE_EQ(received->length, 9.0);
}

TEST_F(FieldDimensionClientTest, RejectsADegenerateField) {
  auto blackboard = startBlackboard("blackboard_degenerate");
  blackboard->declare_parameter("field.size.x", 0.0);
  blackboard->declare_parameter("field.size.y", 6.0);

  std::optional<FieldDimensions> received;
  FieldDimensionClient client(node_, "/blackboard_degenerate", kRetryPeriod,
                              [&received](const FieldDimensions& dimensions) { received = dimensions; });
  spinFor(5 * kRetryPeriod);
  EXPECT_FALSE(client.received());
  EXPECT_FALSE(received.has_value());
}
