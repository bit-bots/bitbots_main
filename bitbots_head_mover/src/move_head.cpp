
#include <chrono>
#include <memory>
#include <bitbots_msgs/msg/joint_command.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <string>
#include <iostream>
#include "head_parameters.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class HeadMover : public rclcpp::Node
{
  //declare publisher and timer
  rclcpp::Publisher<bitbots_msgs::msg::JointCommand>::SharedPtr publisher_;

 //declare subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string head_mode_decision_;




public:
  HeadMover()
  : Node("head_mover")
  {
    publisher_ = this->create_publisher<bitbots_msgs::msg::JointCommand>("head_motor_goals", 10); 
    subscription_ = this->create_subscription<std_msgs::msg::String>( // here we want to call world_model.ball_filtered_callback
      "head_mode", 10, std::bind(&HeadMover::head_mode_callback, this, _1)); // should be callback group 1
      std::cout << "Hello World" << std::endl;
      // load parameters from config
       auto param_listener = std::make_shared<move_head::ParamListener>(rclcpp::Node::make_shared("head_mover")); //is this a problem?
       auto params = param_listener->get_params();
       auto pan_speed = params.look_at.pan_speed;

      //print the param value
      std::cout << "Head pan max speed is: " << pan_speed << std::endl;
  }


private:
//write head_mode_callback
void head_mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  head_mode_decision_ = msg->data.c_str();
  std::cout << "I heard: [" << head_mode_decision_ << "]" << std::endl;
}
//write timer_callback

void head_behavior()
{
//print head_mode_decision_
std::cout << "Head mode is: " << head_mode_decision_ << std::endl;
if (head_mode_decision_ == "LOOK_UP")
{

}};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadMover>());
  rclcpp::shutdown();
  
  return 0;
}
