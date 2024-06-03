#include <bitbots_support_state_detector/walk_support_state_detector.hpp>
namespace bitbots_support_state_detector {

WalkSupportStateDetector::WalkSupportStateDetector()
    : Node("WalkSupportStateDetector"), param_listener_(get_node_parameters_interface()) {
  config_ = param_listener_.get_params();

  // if sim use raw, if not sim use filtered

  pressure_left_sub_ = this->create_subscription<bitbots_msgs::msg::FootPressure>(
      config_.foot_pressure_topic_left, 1, std::bind(&WalkSupportStateDetector::pressure_left_callback, this, _1));
  pressure_right_sub_ = this->create_subscription<bitbots_msgs::msg::FootPressure>(
      config_.foot_pressure_topic_right, 1, std::bind(&WalkSupportStateDetector::pressure_right_callback, this, _1));
  pub_foot_pressure_support_state_ =
      this->create_publisher<biped_interfaces::msg::Phase>("foot_pressure/walk_support_state", 1);

  // if debug, publish a debug for summed pressure
  if (config_.debug) {
    pub_summed_pressure_left_ = this->create_publisher<std_msgs::msg::Float64>("foot_pressure/summed_pressure_left", 1);
    pub_summed_pressure_right_ =
        this->create_publisher<std_msgs::msg::Float64>("foot_pressure/summed_pressure_right", 1);
  }
  curr_stance_.phase = 2;
  pressure_filtered_right_ = 0;
  pressure_filtered_left_ = 0;
  step_duration_right_ = 0;
  up_right_ = this->now();
  step_duration_left_ = 0;
  up_left_ = this->now();
}

void WalkSupportStateDetector::loop() {
  config_ = param_listener_.get_params();
  int curr_stand_left = curr_stand_left_;
  int curr_stand_right = curr_stand_right_;

  biped_interfaces::msg::Phase phase;
  if (curr_stand_left && !curr_stand_right) {
    phase.phase = biped_interfaces::msg::Phase::LEFT_STANCE;
    phase.header.stamp = this->now() + rclcpp::Duration::from_nanoseconds(
                                           int(config_.temporal_step_offset_factor * step_duration_left_));
  } else if (!curr_stand_left && curr_stand_right) {
    phase.phase = biped_interfaces::msg::Phase::RIGHT_STANCE;
    phase.header.stamp = this->now() + rclcpp::Duration::from_nanoseconds(
                                           int(config_.temporal_step_offset_factor * step_duration_right_));
  } else {  // if both are high its double support, but if both are too low, pressure is shared on both feet
    phase.phase = biped_interfaces::msg::Phase::DOUBLE_STANCE;
    phase.header.stamp = this->now() + rclcpp::Duration::from_nanoseconds(
                                           (int(config_.temporal_step_offset_factor * step_duration_left_) +
                                            int(config_.temporal_step_offset_factor * step_duration_right_)) /
                                           2);
  }
  if (phase.phase != curr_stance_.phase) {
    curr_stance_.phase = phase.phase;
    pub_foot_pressure_support_state_->publish(phase);
  }
}

void WalkSupportStateDetector::pressure_left_callback(bitbots_msgs::msg::FootPressure msg) {
  float_t summed_pressure = msg.left_back + msg.left_front + msg.right_front + msg.right_back;
  pressure_filtered_left_ = (1 - config_.k) * summed_pressure + config_.k * pressure_filtered_left_;
  std_msgs::msg::Float64 pressure_msg;
  pressure_msg.data = pressure_filtered_left_;
  if (config_.debug) {
    pub_summed_pressure_left_->publish(pressure_msg);
  }
  if (pressure_filtered_left_ > config_.summed_pressure_threshold_left) {
    if (curr_stand_left_ != true) {
      up_left_ = this->now();
      curr_stand_left_ = true;
    }
  } else {
    if (curr_stand_left_ != false) {
      step_duration_left_ =
          (1 - config_.m) * (this->now().nanoseconds() - up_left_.nanoseconds()) + config_.m * step_duration_left_;
      curr_stand_left_ = false;
    }
  }
}

void WalkSupportStateDetector::pressure_right_callback(bitbots_msgs::msg::FootPressure msg) {
  float_t summed_pressure = msg.left_back + msg.left_front + msg.right_front + msg.right_back;
  pressure_filtered_right_ = (1 - config_.k) * summed_pressure + config_.k * pressure_filtered_right_;
  std_msgs::msg::Float64 pressure_msg;
  pressure_msg.data = pressure_filtered_right_;
  if (config_.debug) {
    pub_summed_pressure_right_->publish(pressure_msg);
  }
  if (pressure_filtered_right_ > config_.summed_pressure_threshold_right) {
    if (curr_stand_right_ != true) {
      up_right_ = this->now();
      curr_stand_right_ = true;
    }
  } else {
    if (curr_stand_right_ != false) {
      step_duration_right_ =
          (1 - config_.m) * (this->now().nanoseconds() - up_right_.nanoseconds()) + config_.m * step_duration_right_;
      curr_stand_right_ = false;
    }
  }
}

}  // namespace bitbots_support_state_detector

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bitbots_support_state_detector::WalkSupportStateDetector>();

  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / 600.0);
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  rclcpp::TimerBase::SharedPtr timer =
      rclcpp::create_timer(node, node->get_clock(), timer_duration, [node]() -> void { node->loop(); });

  exec.spin();
  rclcpp::shutdown();
}
