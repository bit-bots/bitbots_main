#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace bitbots_emergency {
class EMERGENCY_NODE : public rclcpp::Node {
 public:
  explicit EMERGENCY_NODE() : Node("emergency_node") {
    // Create publishers
    client_motor_switch_ = this->create_client<std_srvs::srv::SetBool>("core/switch_power");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EMERGENCY_NODE::_emergencyLoop, this));

    RCLCPP_INFO(this->get_logger(), "Listening for EmergencyButton");
  }

 private:
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_motor_switch_;
  rclcpp::TimerBase::SharedPtr timer_;

  void _emergencyLoop() {
    struct termios oldt, newt;

    RCLCPP_INFO(this->get_logger(), "Loop is active!");
    char ch = '\0';

    int tty_fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);
    tcgetattr(tty_fd, &oldt);
    newt = oldt;
    newt.c_lflag = newt.c_lflag & ~(ICANON | ECHO);
    newt.c_cc[VMIN] = 0;
    newt.c_cc[VTIME] = 0;
    tcsetattr(tty_fd, TCSANOW, &newt);
    read(tty_fd, &ch, 1);
    tcsetattr(tty_fd, TCSANOW, &oldt);
    close(tty_fd);

    if (ch != ' ') {
      RCLCPP_WARN(this->get_logger(), "E-STOP!!!");

      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = false;
      client_motor_switch_->async_send_request(request);
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
