#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <livelybot_msg/msg/power_switch.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace bitbots_emergency {
class EMERGENCY_NODE : public rclcpp::Node {
 public:
  explicit EMERGENCY_NODE() : Node("emergency_node") {
    // Create client
    motor_switch_publisher_ = this->create_publisher<livelybot_msg::msg::PowerSwitch>("/power_switch_control", 1);

    RCLCPP_WARN(this->get_logger(), "Emergency button starting in 3s!");

    std::this_thread::sleep_for(std::chrono::seconds(3));

    RCLCPP_WARN(this->get_logger(), "Listening for EmergencyButton!");

    // repeatedly call loop function
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EMERGENCY_NODE::_emergencyLoop, this));
  }

 private:
  rclcpp::Publisher<livelybot_msg::msg::PowerSwitch>::SharedPtr motor_switch_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void _emergencyLoop() {
    struct termios oldt, newt;

    RCLCPP_INFO(this->get_logger(), "Loop is active!");
    char ch = '\0';

    int tty_fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);  // terminal read only and non-blocking
    tcgetattr(tty_fd, &oldt);
    newt = oldt;
    newt.c_lflag = newt.c_lflag & ~(ICANON | ECHO);  // Don't wait for enter and don't show key in terminal
    newt.c_cc[VMIN] = 0;                             // Don't wait for input
    newt.c_cc[VTIME] = 0;                            // No timeout
    tcsetattr(tty_fd, TCSANOW, &newt);
    read(tty_fd, &ch, 1);
    tcsetattr(tty_fd, TCSANOW, &oldt);
    close(tty_fd);

    if (ch != 'g') {
      RCLCPP_WARN(this->get_logger(), "E-STOP!!!");

      auto msg = livelybot_msg::msg::PowerSwitch();
      msg.power_switch = 0;
      msg.control_switch = 1;
      motor_switch_publisher_->publish(msg);
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
