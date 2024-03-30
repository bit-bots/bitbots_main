#include <bitbots_odometry/walk_support_state_detector.h>
namespace bitbots_odometry {

WalkSupportStateDetector::WalkSupportStateDetector() : Node("WalkSupportStateDetector"),
param_listener_(get_node_parameters_interface())
{
  config_ = param_listener_.get_params();

  // if sim use raw, if not sim use filtered

  pressure_left_sub_ = this->create_subscription<bitbots_msgs::msg::FootPressure>(
        config_.foot_pressure_topic_left, 1, std::bind(&WalkSupportStateDetector::pressure_left_callback, this, _1));
  pressure_right_sub_ = this->create_subscription<bitbots_msgs::msg::FootPressure>(
        config_.foot_pressure_topic_right, 1, std::bind(&WalkSupportStateDetector::pressure_right_callback, this, _1));
  pub_foot_pressure_support_state_ = this->create_publisher<biped_interfaces::msg::Phase>("foot_pressure/walk_support_state", 1);

  // if debug, publish a debug for summed pressure
  if (config_.debug){
    pub_summed_pressure_left_ = this->create_publisher<std_msgs::msg::Float64>("foot_pressure/summed_pressure_left", 1);
    pub_summed_pressure_right_ = this->create_publisher<std_msgs::msg::Float64>("foot_pressure/summed_pressure_right", 1);
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
  
  std::vector<std::pair<float_t, rclcpp::Time>> pressure_times_left = WalkSupportStateDetector::reorder_pressure_array(pressure_left_values_stamped_);
  std::vector<std::pair<float_t, rclcpp::Time>> pressure_times_right = WalkSupportStateDetector::reorder_pressure_array(pressure_right_values_stamped_);
  // get the pressure values
  std::vector<float_t> pressure_left;
  std::vector<float_t> pressure_right;
  for (int i = 0; i < static_cast<int>(pressure_times_left.size()); ++i) {
    pressure_left.push_back(pressure_times_left[i].first);
  }
  for (int i = 0; i <static_cast<int>(pressure_times_right.size()); ++i) {
    pressure_right.push_back(pressure_times_right[i].first);
  }
  // find the inflection points
  int inflection_point_left = WalkSupportStateDetector::findInflectionPoints(pressure_left);
  int inflection_point_right = WalkSupportStateDetector::findInflectionPoints(pressure_right);
    // find the local minima
  int local_minima_left = WalkSupportStateDetector::findLocalMinima(pressure_left);
  int local_minima_right = WalkSupportStateDetector::findLocalMinima(pressure_right);
  if (inflection_point_left != -1){

  left_ts_down_ = pressure_times_left[inflection_point_left].second;
  }
  if (inflection_point_right != -1){
  right_ts_down_ = pressure_times_right[inflection_point_right].second;
  }
  if (local_minima_left != -1){
  right_ts_up_ = pressure_times_left[local_minima_left].second;
  }
  if (local_minima_right != -1){
  left_ts_up_ = pressure_times_right[local_minima_right].second;
  }
  int max_time_stamp_right = std::max(right_ts_down_.nanoseconds(), right_ts_up_.nanoseconds());
  int max_time_stamp_left = std::max(left_ts_down_.nanoseconds(), left_ts_up_.nanoseconds());

  if (static_cast<int>(pressure_right_values_stamped_.size()) > 0){
    while (pressure_right_values_stamped_[0].second.nanoseconds() < max_time_stamp_right){
      pressure_right_values_stamped_.erase(pressure_right_values_stamped_.begin());
    }
  }
  if (static_cast<int>(pressure_left_values_stamped_.size()) > 0){
    while (pressure_left_values_stamped_[0].second.nanoseconds() < max_time_stamp_left){
      pressure_left_values_stamped_.erase(pressure_left_values_stamped_.begin());
    }
  }



  if (right_ts_down_.nanoseconds() > right_ts_up_.nanoseconds()){
    curr_stand_right_ = true;
  }
  else{
    curr_stand_right_ = false;
  }
  if (left_ts_down_.nanoseconds() > left_ts_up_.nanoseconds()){
    curr_stand_left_ = true;
  }
  else{
    curr_stand_left_ = false;
  }


  int curr_stand_left = curr_stand_left_;
  int curr_stand_right = curr_stand_right_;

  biped_interfaces::msg::Phase phase;
    if (curr_stand_left && ! curr_stand_right){
      phase.phase = biped_interfaces::msg::Phase::LEFT_STANCE;
      phase.header.stamp = left_ts_down_;
    }
      else if (!curr_stand_left && curr_stand_right){
      phase.phase = biped_interfaces::msg::Phase::RIGHT_STANCE;
      phase.header.stamp = right_ts_down_;
    }
  else{ // if both are high its double support, but if both are too low, pressure is shared on both feet
      phase.phase = biped_interfaces::msg::Phase::DOUBLE_STANCE;
      if (right_ts_down_.nanoseconds() > left_ts_down_.nanoseconds()){
        phase.header.stamp = right_ts_down_;
      }
      else{

      phase.header.stamp = left_ts_down_;
      }
    }
  if (phase.phase != curr_stance_.phase){
    curr_stance_.phase = phase.phase;
    pub_foot_pressure_support_state_->publish(phase);
  }
  
}

  void WalkSupportStateDetector::pressure_left_callback(bitbots_msgs::msg::FootPressure msg) {
    float_t summed_pressure = (msg.left_back +msg.left_front + msg.right_front + msg.right_back);
    pressure_filtered_left_ = (1-config_.k)*summed_pressure + config_.k*pressure_filtered_left_;
    std_msgs::msg::Float64 pressure_msg;
    pressure_msg.data = pressure_filtered_left_;
    
    // get the time from msg
    rclcpp::Time stamp = msg.header.stamp;
    pub_summed_pressure_left_->publish(pressure_msg);
    pressure_left_values_stamped_.push_back(std::pair<float_t, rclcpp::Time>(pressure_filtered_left_, stamp));
    
    if ( static_cast<int>(pressure_left_values_stamped_.size())> config_.pressure_vector_size){
        pressure_left_values_stamped_.erase(pressure_left_values_stamped_.begin());
    }
    
    // if (pressure_filtered_left_ > config_.summed_pressure_threshold_left){
    //     if (curr_stand_left_ != true){
    //         up_left_ = this->now();
    //         curr_stand_left_ = true;}
    // }
    // else{
    //     if (curr_stand_left_ != false){
    //     step_duration_left_ = (1-config_.m)*(this->now().nanoseconds() - up_left_.nanoseconds()) + config_.m*step_duration_left_;
    //     curr_stand_left_ = false;
    //     }
    // }

  }

  void WalkSupportStateDetector::pressure_right_callback(bitbots_msgs::msg::FootPressure msg) {
    float_t summed_pressure = (msg.left_back +msg.left_front + msg.right_front + msg.right_back);
    pressure_filtered_right_ = (1-config_.k)*summed_pressure + config_.k*pressure_filtered_right_;
    std_msgs::msg::Float64 pressure_msg;
    rclcpp::Time stamp = msg.header.stamp;
    pressure_msg.data = pressure_filtered_right_;
    pressure_right_values_stamped_.push_back(std::pair<float_t, rclcpp::Time>(pressure_filtered_right_, stamp));
    // if one second has passed, start trimming the vector

      pub_summed_pressure_right_->publish(pressure_msg);
    if (static_cast<int>(pressure_right_values_stamped_.size()) > config_.pressure_vector_size){
        pressure_right_values_stamped_.erase(pressure_right_values_stamped_.begin());
    }

      
    // if (pressure_filtered_right_ > config_.summed_pressure_threshold_right){
    //     if (curr_stand_right_ != true){
    //         up_right_ = this->now();
    //         curr_stand_right_ = true;}
    // }
    // else{
    //     if (curr_stand_right_ != false){
    //     step_duration_right_ = (1-config_.m)*(this->now().nanoseconds() - up_right_.nanoseconds()) + config_.m*step_duration_right_;
    //     curr_stand_right_ = false;
    //     }
    // }
  }

int WalkSupportStateDetector::findInflectionPoints(const std::vector<float_t>& function) {
  int steepest_slope_index = -1;
    for (int i = 1; i < static_cast<int>(function.size()) - 1; ++i) {
        double prev_slope = function[i] - function[i - 1];
        double next_slope = function[i] - function[i + 1];
        
        if (prev_slope > 0 && next_slope < 0) {
            return i;
        }
    }
    return -1;
}


// Function to find local minima
int WalkSupportStateDetector::findLocalMinima(const std::vector<float_t>& function) {
  int local_minima_index = -1;
    for (int i = 1; i < static_cast<int>(function.size()) - 1; ++i) {
        double prev_slope = function[i] - function[i - 1];
        double next_slope = function[i] - function[i + 1];
        
        if (prev_slope < 0 && next_slope < 0) {
            if (local_minima_index == -1){
                local_minima_index = i;
            }
            else if (function[i] < function[local_minima_index]){
                local_minima_index = i;
            }
        }
    }
    return local_minima_index;
}

// find the localminima and inflection point of the pressure array
  std::vector<std::pair<float_t, rclcpp::Time>> WalkSupportStateDetector::reorder_pressure_array(std::vector<std::pair<float_t, rclcpp::Time>> pressure_vector) {
    std::vector<std::pair<float_t, rclcpp::Time>> reordered_vector;
    int resample_rate = config_.resample_rate;
    // cast pressure_vector.size() to int
    for (int i = 0; i <static_cast<int>(pressure_vector.size()) ; ++i) {
        if (i % resample_rate == 0) {
            reordered_vector.push_back(pressure_vector[i]);
        }
    }
    return reordered_vector;

}

// make function which trims the pressure vectors to the last second

}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bitbots_odometry::WalkSupportStateDetector>();

  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / 600.0);
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  rclcpp::TimerBase::SharedPtr timer = rclcpp::create_timer(node, node->get_clock(), timer_duration, [node]() -> void {node->loop();});

  exec.spin();
  rclcpp::shutdown();
}
