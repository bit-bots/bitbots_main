#include <chrono>
#include <livelybot_msg/msg/power_switch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>

namespace bitbots_emergency_listener {
class EMERGENCY_NODE_LISTENER : public rclcpp::Node {
 public:
  explicit EMERGENCY_NODE_LISTENER() : Node("emergency_node_listener") {
    e_stop_ = false;

    heartbeat_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "/heartbeat", 10, std::bind(&EMERGENCY_NODE_LISTENER::_emergency_callback, this, std::placeholders::_1));

    motor_switch_publisher_ = this->create_publisher<livelybot_msg::msg::PowerSwitch>("/power_switch_control", 1);

    std::this_thread::sleep_for(std::chrono::seconds(1));

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EMERGENCY_NODE_LISTENER::_watchdog, this));

    timestamp_ = this->now();
  }

 private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heartbeat_subscriber_;
  rclcpp::Publisher<livelybot_msg::msg::PowerSwitch>::SharedPtr motor_switch_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time timestamp_;
  bool e_stop_;

  void _watchdog() {
    if (!e_stop_) {
      rclcpp::Time current_time = this->now();

      rclcpp::Duration delta = current_time - timestamp_;

      if (delta > rclcpp::Duration(std::chrono::milliseconds(500))) {
        _estop();
      }
    }
  }

  void _emergency_callback(std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data == true) {
      timestamp_ = this->now();
    } else if (msg->data == false) {
      _estop();
    }
  }

  void _estop() {
    RCLCPP_WARN(this->get_logger(), "E-STOP!!!");

    auto msg = livelybot_msg::msg::PowerSwitch();
    msg.power_switch = 0;
    msg.control_switch = 1;
    motor_switch_publisher_->publish(msg);

    e_stop_ = true;
  }
};
}  // namespace bitbots_emergency_listener
