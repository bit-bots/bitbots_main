
#include <chrono>
#include <memory>
#include <bitbots_msgs/msg/joint_command.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class HeadMover : public rclcpp::Node
{
public:
  HeadMover()
  : Node("head_mover"), count_(0)
  {
    publisher_ = this->create_publisher<bitbots_msgs::msg::JointCommand>("head_motor_goals", 10); 
    timer_ = this->create_wall_timer(
      500ms, std::bind(&HeadMover::timer_callback, this));
      subscription_ = this->create_subscription<std_msgs::msg::String>( // here we want to call world_model.ball_filtered_callback
      "head_mode", 10, std::bind(&HeadMover::head_mode_callback, this, _1)); // should be callback group 1
  }

private:
  void timer_callback()
  {
    bitbots_msgs::msg::JointCommand message = bitbots_msgs::msg::JointCommand(); // now fill the data somehow?
    RCLCPP_INFO(this->get_logger(), "Publishing..");
    publisher_->publish(message);
  }
    void head_mode_callback(const sensor_msgs::msg::JointState::SharedPtr msg) const // Why SharedPtr?
  {
    RCLCPP_INFO(this->get_logger(), "I heard something..");
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadMover>());
  rclcpp::shutdown();
  return 0;
}
