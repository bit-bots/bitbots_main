#include <bitbots_odometry/walk_support_state_detector.h>
namespace bitbots_odometry {

WalkSupportStateDetector::WalkSupportStateDetector() : Node("WalkSupportStateDetector"),
param_listener_(get_node_parameters_interface())
{
  config_ = param_listener_.get_params();

  // if sim use raw, if not sim use filtered

  pressure_l_sub_ = this->create_subscription<bitbots_msgs::msg::FootPressure>(
        config_.foot_pressure_topic_left, 1, std::bind(&WalkSupportStateDetector::pressure_l_callback, this, _1));
  pressure_r_sub_ = this->create_subscription<bitbots_msgs::msg::FootPressure>(
        config_.foot_pressure_topic_left, 1, std::bind(&WalkSupportStateDetector::pressure_r_callback, this, _1));
  pub_foot_pressure_support_state_ = this->create_publisher<biped_interfaces::msg::Phase>("foot_pressure/walk_support_state", 1);

  // if debug, publish a debug for summed pressure
  if (config_.debug){
    pub_summed_pressure_l_ = this->create_publisher<std_msgs::msg::Float64>("foot_pressure/summed_pressure_left", 1);
    pub_summed_pressure_r_ = this->create_publisher<std_msgs::msg::Float64>("foot_pressure/summed_pressure_right", 1);
  }
  curr_stance_.phase = 2;
  pressure_filtered_right_ = 0;
  pressure_filtered_left_ = 0;
  step_duration_r_ = 0;
  up_r_ = this->now(); 
    step_duration_l_ = 0;
  up_l_ = this->now(); 

}

void WalkSupportStateDetector::loop() {
  config_ = param_listener_.get_params();
  int curr_stand_left = curr_stand_left_;
  int prev_stand_left = prev_stand_left_;

  int curr_stand_right = curr_stand_right_;
  int prev_stand_right = prev_stand_right_;

  biped_interfaces::msg::Phase phase;
    if ( curr_stand_left_ && curr_stand_right_){
      phase.phase = 2;
    }
    else if (curr_stand_left_ && ! curr_stand_right_ && (up_r_ + rclcpp::Duration::from_nanoseconds(int(config_.temporal_step_offset*step_duration_r_))) < this->now()){
      phase.phase = 0;
    }
      else if (!curr_stand_left_ && curr_stand_right_ &&(up_r_ + rclcpp::Duration::from_nanoseconds(int(config_.temporal_step_offset*step_duration_r_))) < this->now()){
      phase.phase = 1;
    }
  if (phase.phase != curr_stance_.phase){
    curr_stance_.phase = phase.phase;
    phase.header.stamp = this->now();

    pub_foot_pressure_support_state_->publish(phase);
  }
  
}

  void WalkSupportStateDetector::pressure_l_callback(bitbots_msgs::msg::FootPressure msg) {
    float_t summed_pressure = msg.left_back +msg.left_front + msg.right_front + msg.right_back;
    pressure_filtered_left_ = (1-config_.k)*summed_pressure + config_.k*pressure_filtered_left_;
    prev_stand_left_ = curr_stand_left_;
    std_msgs::msg::Float64 pressure_msg;
    pressure_msg.data = pressure_filtered_left_;
    if (config_.debug){
      pub_summed_pressure_l_->publish(pressure_msg);
    }
    if (pressure_filtered_left_ > config_.summed_pressure_threshold){
        if (curr_stand_left_ != true){
            up_l_ = this->now();
        
      curr_stand_left_ = true;}
    }
    else{
        if (curr_stand_left_ != false){
        step_duration_l_ = (1-config_.m)*(this->now().nanoseconds() - up_l_.nanoseconds()) + config_.m*step_duration_l_;
      curr_stand_left_ = false;
        }
    }

  }

  void WalkSupportStateDetector::pressure_r_callback(bitbots_msgs::msg::FootPressure msg) {
    float_t summed_pressure = msg.left_back +msg.left_front + msg.right_front + msg.right_back;
    pressure_filtered_right_ = (1-config_.k)*summed_pressure + config_.k*pressure_filtered_right_;
    prev_stand_right_ = curr_stand_right_;
     std_msgs::msg::Float64 pressure_msg;
    pressure_msg.data = pressure_filtered_right_;
    if (config_.debug){
      pub_summed_pressure_r_->publish(pressure_msg);
    }
    if (pressure_filtered_right_ > config_.summed_pressure_threshold){
        if (curr_stand_right_ != true){
            up_r_ = this->now();
        
      curr_stand_right_ = true;}
    }
    else{
        if (curr_stand_right_ != false){
        step_duration_r_ = (1-config_.m)*(this->now().nanoseconds() - up_r_.nanoseconds()) + config_.m*step_duration_r_;
      curr_stand_right_ = false;
        }
    }
  }

}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bitbots_odometry::WalkSupportStateDetector>();

  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / 1200.0);
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  rclcpp::TimerBase::SharedPtr timer = rclcpp::create_timer(node, node->get_clock(), timer_duration, [node]() -> void {node->loop();});

  exec.spin();
  rclcpp::shutdown();
}
