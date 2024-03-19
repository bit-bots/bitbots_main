#include <cmath>
#include <iostream>
#include <memory>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.hpp>

using std::placeholders::_1;

namespace postprocessing {


class PostProcessor {
  std::shared_ptr<rclcpp::Node> node_;

  // Declare subscriber
  //rclcpp::Subscription<bitbots_msgs::msg::HeadMode>::SharedPtr head_mode_subscriber_;
  //rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;

  // Declare publisher
  //rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr position_publisher_;

 public:
  PostProcessor() : node_(std::make_shared<rclcpp::Node>("post_processor")) {
    // Initialize publisher for head motor goals
    //position_publisher_ = node_->create_publisher<bitbots_msgs::msg::JointCommand>("head_motor_goals", 10);

    // Initialize subscriber for head mode
    //head_mode_subscriber_ = node_->create_subscription<bitbots_msgs::msg::HeadMode>(
    //    "head_mode", 10, [this](const bitbots_msgs::msg::HeadMode::SharedPtr msg) { head_mode_callback(msg); });

  }

  /**
   * @brief Callback used to update the head mode
   */
  //void head_mode_callback(const bitbots_msgs::msg::HeadMode::SharedPtr msg) { head_mode_ = msg->head_mode; }

  /**
   * @brief A getter that returns the node
   */
  std::shared_ptr<rclcpp::Node> get_node() { return node_; }
};
}  // namespace move_head

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::experimental::executors::EventsExecutor exec;
  auto post_processor = std::make_shared<postprocessing::PostProcessor>();
  exec.add_node(post_processor->get_node());
  exec.spin();
  rclcpp::shutdown();

  return 0;
}
