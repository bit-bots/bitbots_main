#include <bitbots_ros_control/pressure_converter.hpp>
using std::placeholders::_1;
using std::placeholders::_2;

PressureConverter::PressureConverter(rclcpp::Node::SharedPtr nh, pressure_converter::Params::Common config,
                                     FootConfig foot_config, char side)
    : nh_(nh),
      filtered_pub_(nh_->create_publisher<bitbots_msgs::msg::FootPressure>(foot_config.topic + "/filtered", 1)),
      cop_pub_(nh_->create_publisher<geometry_msgs::msg::PointStamped>("/cop_" + side, 1)),
      // Create wrench publishers
      wrench_pubs_([this](auto &topic) {
        std::array<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr, 5> arr;
        for (size_t i = 0; i < arr.size(); i++) {
          arr[i] = nh_->create_publisher<geometry_msgs::msg::WrenchStamped>(topic + "/wrench/" + wrench_topics_[i], 1);
        }
        return arr;
      }(foot_config.topic)),  // cppcheck-suppress internalAstError
      sub_(nh_->create_subscription<bitbots_msgs::msg::FootPressure>(
          foot_config.topic + "/raw", 1, std::bind(&PressureConverter::pressureCallback, this, _1))),
      tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*nh_)),
      side_(side),
      wrench_frames_([](auto &wrench_topics, auto &side) {
        // Copy array into new array
        std::array<std::string, 4> arr;
        for (size_t i = 0; i < 5; i++) {
          arr[i] = side + "_cleat_" + wrench_topics[i];
        }
        return arr;
      }(wrench_topics_, side)),
      zero_([](const std::vector<double> &vec) {
        // Copy first 4 elements of the config into an new array
        std::array<double, 4> arr;
        std::copy_n(vec.begin(), 4, arr.begin());
        return arr;
      }(foot_config.zero)),
      scale_([](const std::vector<double> &vec) {
        // Copy first 4 elements of the config into an new array
        std::array<double, 4> arr;
        std::copy_n(vec.begin(), 4, arr.begin());
        return arr;
      }(foot_config.scale)),
      previous_values_([](size_t size) {
        std::array<std::vector<double>, 4> arr;
        for (auto &vec : arr) {
          // Initialize vector with size elements, all set to 0
          vec.resize(size, 0);
        }
        return arr;
      }(config.average)),
      average_(config.average),
      calibration_buffer_length_(config.calibration_buffer_length),
      cop_threshold_(config.cop_threshold),
      zero_service_(nh_->create_service<std_srvs::srv::Empty>(
          foot_config.topic + "/set_foot_zero", std::bind(&PressureConverter::zeroCallback, this, _1, _2))),
      scale_service_(nh_->create_service<bitbots_msgs::srv::FootScale>(
          foot_config.topic + "/set_foot_scale", std::bind(&PressureConverter::scaleCallback, this, _1, _2))) {}

void PressureConverter::pressureCallback(bitbots_msgs::msg::FootPressure pressure_raw) {
  bitbots_msgs::msg::FootPressure filtered_msg;

  filtered_msg.header = pressure_raw.header;

  // Add new values to our rolling buffer
  previous_values_[0][current_index_] = ((pressure_raw.left_front) - zero_[0]) * scale_[0];
  previous_values_[1][current_index_] = ((pressure_raw.left_back) - zero_[1]) * scale_[1],
  previous_values_[2][current_index_] = ((pressure_raw.right_front) - zero_[2]) * scale_[2],
  previous_values_[3][current_index_] = ((pressure_raw.right_back) - zero_[3]) * scale_[3];
  current_index_ = (current_index_ + 1) % average_;

  // Calculate the average of the rolling buffer
  filtered_msg.left_front = std::accumulate(previous_values_[0].begin(), previous_values_[0].end(), 0.0) / average_;
  filtered_msg.left_back = std::accumulate(previous_values_[1].begin(), previous_values_[1].end(), 0.0) / average_;
  filtered_msg.right_front = std::accumulate(previous_values_[2].begin(), previous_values_[2].end(), 0.0) / average_;
  filtered_msg.right_back = std::accumulate(previous_values_[3].begin(), previous_values_[3].end(), 0.0) / average_;

  // Clip to zero
  filtered_msg.left_front = std::max(filtered_msg.left_front, 0.0);
  filtered_msg.left_back = std::max(filtered_msg.left_back, 0.0);
  filtered_msg.right_front = std::max(filtered_msg.right_front, 0.0);
  filtered_msg.right_back = std::max(filtered_msg.right_back, 0.0);

  std::vector<double> forces_list = {filtered_msg.left_front, filtered_msg.left_back, filtered_msg.right_front,
                                     filtered_msg.right_back};
  for (int i = 0; i < 4; i++) {
    geometry_msgs::msg::WrenchStamped w;
    w.header.frame_id = wrench_frames_[i];
    w.header.stamp = pressure_raw.header.stamp;
    w.wrench.force.z = forces_list[i];
    wrench_pubs_[i]->publish(w);
  }
  filtered_pub_->publish(filtered_msg);

  // Store zero and scale values if we are in calibration mode
  if (save_calibration_buffer_) {
    calibration_buffer_[0].push_back(pressure_raw.left_front);
    calibration_buffer_[1].push_back(pressure_raw.left_back);
    calibration_buffer_[2].push_back(pressure_raw.right_front);
    calibration_buffer_[3].push_back(pressure_raw.right_back);
  }

  // Publish center of pressure
  double pos_x = 0.085, pos_y = 0.045;
  geometry_msgs::msg::PointStamped cop;
  cop.header.frame_id = side_ + "_sole";
  cop.header.stamp = pressure_raw.header.stamp;
  double sum_of_forces =
      filtered_msg.left_front + filtered_msg.left_back + filtered_msg.right_back + filtered_msg.right_front;
  if (sum_of_forces > cop_threshold_) {
    cop.point.x =
        (filtered_msg.left_front + filtered_msg.right_front - filtered_msg.left_back - filtered_msg.right_back) *
        pos_x / sum_of_forces;
    cop.point.x = std::max(std::min(cop.point.x, pos_x), -pos_x);

    cop.point.y =
        (filtered_msg.left_front + filtered_msg.left_back - filtered_msg.right_front - filtered_msg.right_back) *
        pos_y / sum_of_forces;
    cop.point.y = std::max(std::min(cop.point.y, pos_y), -pos_y);
  } else {
    cop.point.x = 0;
    cop.point.y = 0;
  }
  cop_pub_->publish(cop);

  geometry_msgs::msg::TransformStamped cop_tf;
  cop_tf.header = cop.header;
  cop_tf.child_frame_id = "cop_" + side_;
  cop_tf.transform.translation.x = cop.point.x;
  cop_tf.transform.translation.y = cop.point.y;
  cop_tf.transform.rotation.w = 1;
  tf_broadcaster_->sendTransform(cop_tf);

  geometry_msgs::msg::WrenchStamped w_cop;
  w_cop.header.frame_id = "cop_" + side_;
  w_cop.header.stamp = pressure_raw.header.stamp;
  w_cop.wrench.force.z = sum_of_forces;
  wrench_pubs_[4]->publish(w_cop);
}

void PressureConverter::resetZeroAndScaleValues() {
  // Reset calibration buffer
  for (auto &cleat : calibration_buffer_) {
    cleat.clear();
  }
}

void PressureConverter::collectMessages() {
  save_calibration_buffer_ = true;
  while (int(calibration_buffer_[0].size()) < calibration_buffer_length_) {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 20, "%ld of %d msgs", calibration_buffer_[0].size(),
                         calibration_buffer_length_);
  }
  save_calibration_buffer_ = false;
}

void PressureConverter::showYAML() {
  YAML::Emitter e;
  e << YAML::Key << "zero";
  e << YAML::Value << YAML::Flow << zero_;
  e << YAML::Key << "scale";
  e << YAML::Value << YAML::Flow << scale_;

  RCLCPP_INFO_STREAM(nh_->get_logger(),
                     "The following calibration values are the new calibration values:" << std::endl
                                                                                        << e.c_str());
}

bool PressureConverter::scaleCallback(const std::shared_ptr<bitbots_msgs::srv::FootScale::Request> req,
                                      std::shared_ptr<bitbots_msgs::srv::FootScale::Response> resp) {
  collectMessages();
  double average =
      std::accumulate(calibration_buffer_[req->sensor].begin(), calibration_buffer_[req->sensor].end(), 0.0) /
      calibration_buffer_[req->sensor].size();
  RCLCPP_WARN_STREAM(nh_->get_logger(), "avg: " << average);
  average -= zero_[req->sensor];
  RCLCPP_WARN_STREAM(nh_->get_logger(), "avg_after: " << average);

  scale_[req->sensor] = req->weight / average;
  resetZeroAndScaleValues();
  showYAML();
  return true;
}

bool PressureConverter::zeroCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> resp) {
  collectMessages();
  for (int i = 0; i < 4; i++) {
    zero_[i] = std::accumulate(calibration_buffer_[i].begin(), calibration_buffer_[i].end(), 0.0) /
               calibration_buffer_[i].size();
  }
  resetZeroAndScaleValues();
  showYAML();
  return true;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // Create node and executor
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("pressure_converter");
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(nh);

  // Get config
  auto param_listener = pressure_converter::ParamListener(nh);
  auto config = param_listener.get_params();

  // Pass config to PressureConverters
  PressureConverter r(nh, config.common, FootConfig(config.right), 'r');
  PressureConverter l(nh, config.common, FootConfig(config.left), 'l');

  executor.spin();

  return 0;
}
