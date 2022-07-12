#include <signal.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <bitbots_msgs/msg/buttons.hpp>
#include <humanoid_league_msgs/msg/audio.hpp>
#include <std_msgs/msg/bool.hpp>
#include <bitbots_msgs/srv/manual_penalize.hpp>
#include <test_msgs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;
namespace bitbots_buttons {

class ButtonNode : public rclcpp::Node {
public:
  explicit ButtonNode(): Node("buttons",rclcpp::NodeOptions()) {
      
    this->declare_parameter<bool>("speak_active", true);
    this->declare_parameter<double>("short_time", 2.0);
    this->declare_parameter<bool>("manual_penalty", true);
    this->declare_parameter<bool>("in_game", true);
    this->declare_parameter<double>("debounce_time", 0.1);
    this->declare_parameter<bool>("speak", true);

    this->get_parameter("speak_active", speaking_active_);
    this->get_parameter("short_time", short_time_);
    this->get_parameter("manual_penalty", manual_penalty_mode_);
    this->get_parameter("in_game", in_game_);
    this->get_parameter("debounce_time", debounce_time_);
    this->get_parameter("speak", speak_);

    // --- Class variables ---
    button1_ = false;
    button2_ = false;
    button3_ = false;
    button1_time_ = 0.0;
    button2_time_ = 0.0;
    button3_time_ = 0.0;

    // --- Initialize Topics ---
    speak_pub_ = this->create_publisher<humanoid_league_msgs::msg::Audio>("/speak", 1);
    shoot_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/shoot_button", 1);

    if (manual_penalty_mode_){
      manual_penalize_client_ = this->create_client<bitbots_msgs::srv::ManualPenalize>("manual_penalize");
      while (!manual_penalize_client_->wait_for_service(3s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the manual penalize service. Exiting.");
          exit(0);
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }
    }
     
    if(!in_game_){
      foot_zero_client_ = this->create_client<test_msgs::srv::Empty>("manual_penalize");
      while (!foot_zero_client_->wait_for_service(3s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the foot zero service. Exiting.");
          exit(0);
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }
    }

    power_client_ = this->create_client<std_srvs::srv::SetBool>("/core/switch_power");
    while (!power_client_->wait_for_service(3s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the power switch service. Exiting.");
        exit(0);
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }

    buttons_sub_ = this->create_subscription<bitbots_msgs::msg::Buttons>("/buttons", 1, std::bind(&bitbots_buttons::ButtonNode::buttonCb, this, _1));
  }

  void buttonCb(const bitbots_msgs::msg::Buttons::SharedPtr msg){
    if(msg->button1 && !button1_){
      button1_ = true;
      button1_time_ = this->get_clock()->now().seconds();
    }else if(msg->button2 && !button2_){
      button2_ = true;
      button2_time_ = this->get_clock()->now().seconds();
    }else if(msg->button3 && !button3_){
      button3_ = true;
      button3_time_ = this->get_clock()->now().seconds();
    }else if(!msg->button1 && button1_){
      // button 1 not pressed anymore
      button1_ = false;
      double current_time = this->get_clock()->now().seconds();
      if (current_time - button1_time_ > debounce_time_){
        if(current_time - button1_ < short_time_ || in_game_){
          // button 1 short
          speak("1 short");
          setPower(false);
        }else{
          // button 1 long
          speak("1 long");
          setPower(true);
        }
      }
      button1_time_ = 0;
    }else if (! msg->button2 && button2_){
      // button2 not pressed anymore
      button2_ = false;
      double current_time = this->get_clock()->now().seconds();
      if (current_time - button2_time_ > debounce_time_){
        if(current_time - button2_ < short_time_ || in_game_){
          speak("2 short");
          setPenalty(true);
        } else {
          speak("2 long");
          setPenalty(true);
        }
      }
      button2_time_ = 0;
    }else if (! msg->button3 && button3_){
      // button2 not pressed anymore
      button3_ = false;
      double current_time = this->get_clock()->now().seconds();
      if (current_time - button3_time_ > debounce_time_){
        if(current_time - button3_ < short_time_ || in_game_){
          speak("3 short");
          setPenalty(false);
        } else {
          speak("3 short");
          setPenalty(false);
        }
      }
      button3_time_ = 0;
    }
  }

  void setPenalty(bool penalize){
    // Penalizes the robot, if it is not penalized and manual penalty mode is true.
    if(manual_penalty_mode_){
      // switch penalty state by calling service on HCM
        auto request = std::make_shared<bitbots_msgs::srv::ManualPenalize::Request>();
        request->penalize = penalize;
        auto result = manual_penalize_client_->async_send_request(request);
    }
  }

  void zeroFootSensors(){
    // Zeroes foot sensors, if foot zero mode is true
    if(!in_game_){
      auto request = std::make_shared<test_msgs::srv::Empty::Request>();
      auto result = foot_zero_client_->async_send_request(request);
    }
  }

  void setPower(bool power){
    // Set power to the servos
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = power;
    auto result = power_client_->async_send_request(request);
  }

private:
  void speak(std::string text) {
    /**
      *  Helper method to send a message for text-to-speech output
      */
    if(speak_){
      humanoid_league_msgs::msg::Audio msg = humanoid_league_msgs::msg::Audio();
      msg.text = text;
      msg.priority = 100;
      speak_pub_->publish(msg);
    }
  }
  
  bool speaking_active_;
  double short_time_;
  bool manual_penalty_mode_;
  bool in_game_;
  double debounce_time_;
  bool speak_;

  bool button1_;
  bool button2_;
  bool button3_;
  double button1_time_;
  double button2_time_;
  double button3_time_;

  rclcpp::Publisher<humanoid_league_msgs::msg::Audio>::SharedPtr speak_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoot_publisher_;
  rclcpp::Client<bitbots_msgs::srv::ManualPenalize>::SharedPtr manual_penalize_client_;
  rclcpp::Client<test_msgs::srv::Empty>::SharedPtr foot_zero_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr power_client_;
  rclcpp::Subscription<bitbots_msgs::msg::Buttons>::SharedPtr buttons_sub_;
};
}// namespace bitbots_buttons

int main(int argc, char* argv[]) {
  // initialize ros
  rclcpp::init(argc, argv);
  auto buttons_node = std::make_shared<bitbots_buttons::ButtonNode>();

  rclcpp::executors::EventsExecutor exec;
  exec.add_node(buttons_node);
  exec.spin();
  rclcpp::shutdown();
}
