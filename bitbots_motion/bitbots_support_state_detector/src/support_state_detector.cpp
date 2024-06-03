#include <biped_interfaces/msg/phase.hpp>
#include <bitbots_msgs/msg/foot_pressure.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "support_state_detector_parameters.hpp"

using std::placeholders::_1;

class SupportStateDetector : public rclcpp::Node {
 public:
  SupportStateDetector()
      : Node("SupportStateDetector"),
        param_listener_(get_node_parameters_interface()),
        config_(param_listener_.get_params()),
        pressure_left_sub_(this->create_subscription<bitbots_msgs::msg::FootPressure>(
            config_.foot_pressure_topic_left, 1, std::bind(&SupportStateDetector::pressure_left_callback, this, _1))),
        pressure_right_sub_(this->create_subscription<bitbots_msgs::msg::FootPressure>(
            config_.foot_pressure_topic_right, 1, std::bind(&SupportStateDetector::pressure_right_callback, this, _1))),
        pub_foot_pressure_support_state_(
            this->create_publisher<biped_interfaces::msg::Phase>("foot_pressure/walk_support_state", 1)),
        pub_summed_pressure_left_(
            this->create_publisher<std_msgs::msg::Float64>("foot_pressure/summed_pressure_left", 1)),
        pub_summed_pressure_right_(
            this->create_publisher<std_msgs::msg::Float64>("foot_pressure/summed_pressure_right", 1)) {
    curr_stance_.phase = biped_interfaces::msg::Phase::DOUBLE_STANCE;
  }

  void loop() {
    // Update parameters if they have changed
    if (param_listener_.is_old(config_)) {
      config_ = param_listener_.get_params();
    }

    // Build the phase message based on the current stance
    biped_interfaces::msg::Phase phase;
    if (curr_stand_left_ and !curr_stand_right_) {
      phase.phase = biped_interfaces::msg::Phase::LEFT_STANCE;
    } else if (!curr_stand_left_ and curr_stand_right_) {
      phase.phase = biped_interfaces::msg::Phase::RIGHT_STANCE;
    } else {  // if both are high its double support, but if both are too low, pressure is shared on both feet
      phase.phase = biped_interfaces::msg::Phase::DOUBLE_STANCE;
    }

    // Only send message if phase has changed
    if (phase.phase != curr_stance_.phase) {
      // Add timestamp to message
      phase.header.stamp = this->now();
      // Store current phase
      curr_stance_.phase = phase.phase;
      // Publish phase
      pub_foot_pressure_support_state_->publish(phase);
    }
  }

 private:
  void pressure_left_callback(bitbots_msgs::msg::FootPressure msg) {
    float_t summed_pressure = msg.left_back + msg.left_front + msg.right_front + msg.right_back;
    pressure_filtered_left_ = (1 - config_.k) * summed_pressure + config_.k * pressure_filtered_left_;

    curr_stand_left_ = pressure_filtered_left_ > config_.summed_pressure_threshold_left;

    if (config_.debug) {
      std_msgs::msg::Float64 pressure_msg;
      pressure_msg.data = pressure_filtered_left_;
      pub_summed_pressure_left_->publish(pressure_msg);
    }
  }

  void pressure_right_callback(bitbots_msgs::msg::FootPressure msg) {
    float_t summed_pressure = msg.left_back + msg.left_front + msg.right_front + msg.right_back;
    pressure_filtered_right_ = (1 - config_.k) * summed_pressure + config_.k * pressure_filtered_right_;

    curr_stand_right_ = pressure_filtered_right_ > config_.summed_pressure_threshold_right;

    if (config_.debug) {
      std_msgs::msg::Float64 pressure_msg;
      pressure_msg.data = pressure_filtered_right_;
      pub_summed_pressure_right_->publish(pressure_msg);
    }
  }

  // Declare parameter listener and struct from the generate_parameter_library
  support_state_detector::ParamListener param_listener_;
  support_state_detector::Params config_;

  // Declare subscriptions and publishers
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_left_sub_;
  rclcpp::Subscription<bitbots_msgs::msg::FootPressure>::SharedPtr pressure_right_sub_;
  rclcpp::Publisher<biped_interfaces::msg::Phase>::SharedPtr pub_foot_pressure_support_state_;
  // if debug is true, publish a debug for summed pressure
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_summed_pressure_left_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_summed_pressure_right_;

  // Declare variables
  bool curr_stand_left_ = false;
  bool curr_stand_right_ = false;
  float_t pressure_filtered_left_ = 0;
  float_t pressure_filtered_right_ = 0;
  biped_interfaces::msg::Phase curr_stance_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SupportStateDetector>();

  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / 500.0);
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  rclcpp::TimerBase::SharedPtr timer =
      rclcpp::create_timer(node, node->get_clock(), timer_duration, [node]() -> void { node->loop(); });

  exec.spin();
  rclcpp::shutdown();
}
