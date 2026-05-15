#include <chrono>
#include <livelybot_msg/msg/power_switch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>

namespace bitbots_emergency_listener {
class EMERGENCY_NODE_LISTENER : public rclcpp::Node {
 public:
  explicit EMERGENCY_NODE_LISTENER() : Node("emergency_node_listener") {
    e_stop_ = false;         // flag for reducing noise
    start_trigger_ = false;  // starting trigger flag

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
  bool start_trigger_;

  void _watchdog() {
    if (!e_stop_) {
      RCLCPP_INFO(this->get_logger(), "Loop");

      rclcpp::Time current_time = this->now();

      if (start_trigger_) {
        RCLCPP_INFO(this->get_logger(), "Start is triggered!");
        rclcpp::Duration delta = current_time - timestamp_;

        if (delta > rclcpp::Duration(
                        std::chrono::milliseconds(500))) {  // triggers e-stop if connectionloss is longer than 500ms
          _estop();
        }
      }
    }
  }

  void _emergency_callback(std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data == true) {
      start_trigger_ = true;  // triggers start when first heartbeat arrives
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

void thread_spin(rclcpp::experimental::executors::EventsExecutor::SharedPtr executor) { executor->spin(); }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<bitbots_emergency_listener::EMERGENCY_NODE_LISTENER>();

  rclcpp::experimental::executors::EventsExecutor::SharedPtr exec =
      std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
  exec->add_node(node);
  std::thread thread_obj(thread_spin, exec);

  // Join the thread
  thread_obj.join();

  rclcpp::shutdown();
}
