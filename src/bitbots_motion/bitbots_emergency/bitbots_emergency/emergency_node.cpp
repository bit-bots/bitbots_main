#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>

// cppcheck-suppress missingInclude
#include "std_msgs/msg/bool.hpp"

namespace bitbots_emergency {
class EMERGENCY_NODE : public rclcpp::Node {
 public:
  explicit EMERGENCY_NODE() : Node("emergency_node") {
    // Create publishers
    pub_motor_switch_ = this->create_publisher<std_msgs::msg::Bool>("core/switch_power", 1);

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EMERGENCY_NODE::_emergencyButtonLoop, this));
  }

 private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_motor_switch_;
  rclcpp::TimerBase::SharedPtr timer_;

  char _getKeyNonBlocking() {
    struct termios oldt, newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag = newt.c_lflag & ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    char ch = '\0';
    read(STDIN_FILENO, &ch, 1);
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
  }

  void _emergencyButtonLoop() {
    char emergencyButton = ' ';

    char ch = _getKeyNonBlocking();

    if (emergencyButton == ch) {
      auto msg = std_msgs::msg::Bool();
      msg.data = true;
      pub_motor_switch_->publish(msg);
    }
  }
};
}  // namespace bitbots_emergency

void thread_spin(rclcpp::experimental::executors::EventsExecutor::SharedPtr executor) { executor->spin(); }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<bitbots_emergency::EMERGENCY_NODE>();

  rclcpp::experimental::executors::EventsExecutor::SharedPtr exec =
      std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
  exec->add_node(node);
  std::thread thread_obj(thread_spin, exec);

  // Join the thread
  thread_obj.join();

  rclcpp::shutdown();
}
