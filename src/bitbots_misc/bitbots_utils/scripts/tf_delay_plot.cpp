// This script logs the delay of certain tf frames and publishes them to be visualized in plotjuggler.

#include <chrono>
#include <iterator>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <thread>
#include <vector>

using namespace std::placeholders;

class TfDelayPlot : public rclcpp::Node {
 public:
  TfDelayPlot() : Node("tf_delay_plot", rclcpp::NodeOptions()) {
    RCLCPP_INFO(this->get_logger(), "Starting tf_delay_plot");
    // Create correct qos profile
    tf_sub_ =
        this->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 5, std::bind(&TfDelayPlot::tf_callback, this, _1));
  }

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Got transform message");
    for (auto &transform : msg->transforms) {
      // Calculate delay
      double delay =
          ((transform.header.stamp.sec * 1.0e9 + transform.header.stamp.nanosec) - this->now().nanoseconds()) / 1.0e9;

      if (std::abs(delay) > 0.0001) {
        // Print
        RCLCPP_INFO(this->get_logger(), "Frame: %s, Delay: %f", transform.child_frame_id.c_str(), delay);

        // Print current time
        RCLCPP_INFO(this->get_logger(), "Current time: %f", this->now().seconds());

        // Print stamp
        RCLCPP_INFO(this->get_logger(), "Stamp: %f",
                    transform.header.stamp.sec + transform.header.stamp.nanosec / 1.0e9);
      }
    }
  }

 private:
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfDelayPlot>());
  rclcpp::shutdown();
  return 0;
}
