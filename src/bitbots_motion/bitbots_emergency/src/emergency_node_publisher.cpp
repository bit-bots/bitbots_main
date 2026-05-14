#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <thread>

namespace bitbots_emergency_publisher {
class EMERGENCY_NODE_PUBLISHER : public rclcpp::Node {
 public:
  explicit EMERGENCY_NODE_PUBLISHER() : Node("emergency_node_publisher") {
    // Create client
    heartbeat_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/heartbeat", 1);

    RCLCPP_WARN(this->get_logger(), "Emergency button starting in 3s!");

    std::this_thread::sleep_for(std::chrono::seconds(3));

    RCLCPP_WARN(this->get_logger(), "Listening for EmergencyButton!");

    tty_fd_ = open("/dev/tty", O_RDONLY | O_NONBLOCK);  // terminal read only and non-blocking

    // repeatedly call loop function
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&EMERGENCY_NODE_PUBLISHER::_emergencyLoop, this));
  }

  ~EMERGENCY_NODE_PUBLISHER() { close(tty_fd_); }

 private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr heartbeat_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int tty_fd_;

  void _emergencyLoop() {
    struct termios oldt, newt;

    RCLCPP_INFO(this->get_logger(), "Loop is active!");
    char buf[64];

    tcgetattr(tty_fd_, &oldt);
    newt = oldt;
    newt.c_lflag = newt.c_lflag & ~(ICANON | ECHO);  // Don't wait for enter and don't show key in terminal
    newt.c_cc[VMIN] = 0;                             // Don't wait for input
    newt.c_cc[VTIME] = 0;                            // No timeout
    tcsetattr(tty_fd_, TCSANOW, &newt);

    ssize_t n = read(tty_fd_, buf, sizeof(buf));
    int cnt = 0;
    for (ssize_t i = 0; i < n; i++) {
      if (buf[i] != ' ') {
        cnt = cnt + 1;
      }
    }

    auto msg = std_msgs::msg::Bool();
    if (cnt == n) {
      RCLCPP_WARN(this->get_logger(), "Sending E-STOP signal!!!");

      msg.data = false;  // E-stop!
    } else {
      msg.data = true;  // Robot should function
    }
    heartbeat_publisher_->publish(msg);

    tcsetattr(tty_fd_, TCSANOW, &oldt);
  }
};
}  // namespace bitbots_emergency_publisher

void thread_spin(rclcpp::experimental::executors::EventsExecutor::SharedPtr executor) { executor->spin(); }

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<bitbots_emergency_publisher::EMERGENCY_NODE_PUBLISHER>();

  rclcpp::experimental::executors::EventsExecutor::SharedPtr exec =
      std::make_shared<rclcpp::experimental::executors::EventsExecutor>();
  exec->add_node(node);
  std::thread thread_obj(thread_spin, exec);

  // Join the thread
  thread_obj.join();

  rclcpp::shutdown();
}
