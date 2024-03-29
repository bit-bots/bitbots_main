#include <signal.h>

#include <bitbots_msgs/msg/audio.hpp>
#include <bitbots_msgs/msg/buttons.hpp>
#include <bitbots_msgs/srv/manual_penalize.hpp>
#include <bitbots_msgs/srv/set_teaching_mode.hpp>
#include <game_controller_hl_interfaces/msg/game_state.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <thread>

using std::placeholders::_1;
using namespace std::chrono_literals;
namespace bitbots_buttons {

class ButtonNode : public rclcpp::Node {
 public:
  explicit ButtonNode() : Node("buttons", rclcpp::NodeOptions()) {
    this->declare_parameter<bool>("speak_active", true);
    this->declare_parameter<double>("short_time", 2.0);
    this->declare_parameter<bool>("manual_penalty", true);
    this->declare_parameter<double>("debounce_time", 0.1);
    this->declare_parameter<bool>("speak", true);

    this->get_parameter("speak_active", speaking_active_);
    this->get_parameter("short_time", short_time_);
    this->get_parameter("manual_penalty", manual_penalty_mode_);
    this->get_parameter("debounce_time", debounce_time_);
    this->get_parameter("speak", speak_);

    // --- Initialize Topics ---
    speak_pub_ = this->create_publisher<bitbots_msgs::msg::Audio>("/speak", 1);

    if (manual_penalty_mode_) {
      manual_penalize_client_ = this->create_client<bitbots_msgs::srv::ManualPenalize>("manual_penalize");
      manual_penalty_mode_ = manual_penalize_client_->wait_for_service(3s);
    }

    foot_zero_client_ = this->create_client<std_srvs::srv::Empty>("set_foot_zero");
    foot_zero_available_ = foot_zero_client_->wait_for_service(3s);

    power_client_ = this->create_client<std_srvs::srv::SetBool>("/core/switch_power");
    while (!power_client_->wait_for_service(3s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the power switch service. Exiting.");
        exit(0);
      }
      RCLCPP_INFO(this->get_logger(), "service switch_power not available, waiting again...");
    }

    teaching_mode_client_ = this->create_client<bitbots_msgs::srv::SetTeachingMode>("teaching_mode");
    buttons_sub_ = this->create_subscription<bitbots_msgs::msg::Buttons>(
        "/buttons", 1, std::bind(&bitbots_buttons::ButtonNode::buttonCb, this, _1));
    gamestate_sub_ = this->create_subscription<game_controller_hl_interfaces::msg::GameState>(
        "gamestate", 1, std::bind(&bitbots_buttons::ButtonNode::gamestateCb, this, _1));
  }

  // Sets the in_game_ variable to true, if a Gamestate message from the Gamecontroller arrives.
  void gamestateCb(const game_controller_hl_interfaces::msg::GameState::SharedPtr msg) { in_game_ = true; }

  void buttonCb(const bitbots_msgs::msg::Buttons::SharedPtr msg) {
    // button1 - red
    // button2 - green
    // button3 - blue

    if (msg->button1 && !button1_) {
      button1_ = true;
      button1_time_ = this->get_clock()->now().seconds();
    } else if (msg->button2 && !button2_) {
      button2_ = true;
      button2_time_ = this->get_clock()->now().seconds();
    } else if (msg->button3 && !button3_) {
      button3_ = true;
      button3_time_ = this->get_clock()->now().seconds();
    } else if (!msg->button1 && button1_) {
      // button 1 not pressed anymore
      button1_ = false;
      double current_time = this->get_clock()->now().seconds();
      if (current_time - button1_time_ > debounce_time_) {
        if (current_time - button1_time_ < short_time_ || in_game_) {
          // button 1 short
          speak("Red button pressed short. Turning motor power off.");
          setPower(false);  // this can not be reversed because the button cuts the power of himself
        } else {
          // button 1 long
          speak("Red button pressed long. No action implemented.");
        }
      }
      button1_time_ = 0;
    } else if (!msg->button2 && button2_) {
      // button2 not pressed anymore
      button2_ = false;
      double current_time = this->get_clock()->now().seconds();
      if (current_time - button2_time_ > debounce_time_) {
        if (current_time - button2_time_ < short_time_ || in_game_) {
          speak("Green button pressed short. Try deactivating Penalty mode");
          setPenalty(false);
        } else {
          speak("Green button pressed long. Try deactivating teaching mode");
          // Turn teaching mode off
          setTeachingMode(false);
        }
      }
      button2_time_ = 0;
    } else if (!msg->button3 && button3_) {
      // button2 not pressed anymore
      button3_ = false;
      double current_time = this->get_clock()->now().seconds();
      if (current_time - button3_time_ > debounce_time_) {
        if (current_time - button3_time_ < short_time_ || in_game_) {
          speak("Blue button pressed short. Try activating Penalty mode");
          setPenalty(true);
        } else {
          speak("Blue button pressed long. Try switching teaching mode state");
          // Turn teaching mode on or switch between HOLD and TEACH
          setTeachingMode(true);
        }
      }
      button3_time_ = 0;
    }
  }

  void setTeachingMode(bool state) {
    auto request = std::make_shared<bitbots_msgs::srv::SetTeachingMode::Request>();
    if (state) {
      // switch teaching mode state to SWITCH
      request->state = bitbots_msgs::srv::SetTeachingMode::Request::SWITCH;
    } else {
      // switch teaching mode state to OFF
      request->state = bitbots_msgs::srv::SetTeachingMode::Request::OFF;
    }
    teaching_mode_client_->async_send_request(request);
  }

  void setPenalty(bool penalize) {
    // Penalizes the robot, if it is not penalized and manual penalty mode is true.
    if (manual_penalty_mode_) {
      // switch penalty state by calling service on HCM
      auto request = std::make_shared<bitbots_msgs::srv::ManualPenalize::Request>();
      request->penalize = penalize;
      manual_penalize_client_->async_send_request(request);
    }
  }

  void zeroFootSensors() {
    // Zeroes foot sensors, if foot zero mode is true
    if (foot_zero_available_) {
      if (!in_game_) {
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        foot_zero_client_->async_send_request(request);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "service not available");
    }
  }

  void setPower(bool power) {
    // Set power to the servos
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = power;
    power_client_->async_send_request(request);
  }

 private:
  void speak(const std::string& text) {
    /**
     *  Helper method to send a message for text-to-speech output
     */
    if (speak_) {
      bitbots_msgs::msg::Audio msg = bitbots_msgs::msg::Audio();
      msg.text = text;
      msg.priority = 100;
      speak_pub_->publish(msg);
    }
  }

  bool speaking_active_;
  double short_time_;
  bool manual_penalty_mode_;
  bool in_game_ = false;
  double debounce_time_;
  bool speak_;

  bool button1_ = false;
  bool button2_ = false;
  bool button3_ = false;
  bool foot_zero_available_;
  double button1_time_ = 0.0;
  double button2_time_ = 0.0;
  double button3_time_ = 0.0;

  rclcpp::Publisher<bitbots_msgs::msg::Audio>::SharedPtr speak_pub_;
  rclcpp::Client<bitbots_msgs::srv::ManualPenalize>::SharedPtr manual_penalize_client_;
  rclcpp::Client<bitbots_msgs::srv::SetTeachingMode>::SharedPtr teaching_mode_client_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr foot_zero_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr power_client_;
  rclcpp::Subscription<bitbots_msgs::msg::Buttons>::SharedPtr buttons_sub_;
  rclcpp::Subscription<game_controller_hl_interfaces::msg::GameState>::SharedPtr gamestate_sub_;
};
}  // namespace bitbots_buttons

int main(int argc, char* argv[]) {
  // initialize ros
  rclcpp::init(argc, argv);
  auto buttons_node = std::make_shared<bitbots_buttons::ButtonNode>();

  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(buttons_node);
  exec.spin();
  rclcpp::shutdown();
}
