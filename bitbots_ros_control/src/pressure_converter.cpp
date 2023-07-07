#include <bitbots_ros_control/pressure_converter.h>
using std::placeholders::_1;
using std::placeholders::_2;

PressureConverter::PressureConverter(rclcpp::Node::SharedPtr nh, char side) {
  nh_ = nh;
  std::string topic;
  rclcpp::CallbackGroup::SharedPtr sub_cbg_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (side == 'l') {
    if (!nh_->has_parameter("left_topic"))
      RCLCPP_ERROR_STREAM(nh_->get_logger(), nh_->get_name() << ": left_topic not specified");
    topic = nh_->get_parameter("left_topic").as_string();
  } else if (side == 'r') {
    if (!nh_->has_parameter("right_topic"))
      RCLCPP_ERROR_STREAM(nh_->get_logger(), nh_->get_name() << ": right_topic not specified");
    topic = nh_->get_parameter("right_topic").as_string();
  }

  if (!nh_->has_parameter("average"))
    RCLCPP_ERROR_STREAM(nh_->get_logger(), nh_->get_name() << ": average not specified");
  average_ = nh_->get_parameter("average").as_int();

  if (!nh_->has_parameter("scale_and_zero_average"))
    RCLCPP_ERROR_STREAM(nh_->get_logger(), nh_->get_name() << ": scale_and_zero_average not specified");
  scale_and_zero_average_ = nh_->get_parameter("scale_and_zero_average").as_int();

  scale_lr_ = "scale_";
  scale_lr_ += side;
  zero_lr_ = "zero_";
  zero_lr_ += side;
  cop_lr_ = "cop_";
  cop_lr_ += side;
  sole_lr_ = side;
  sole_lr_ += "_sole";

  if (!nh_->has_parameter(zero_lr_)) {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), nh_->get_name() << ": " << zero_lr_ << " not specified");
    zero_ = {0, 0, 0, 0};
  } else {
    zero_ = nh_->get_parameter(zero_lr_).as_double_array();
  }

  if (!nh_->has_parameter(scale_lr_)) {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), nh_->get_name() << ": " << scale_lr_ << " not specified");
    scale_ = {1, 1, 1, 1};
  } else {
    scale_ = nh_->get_parameter(scale_lr_).as_double_array();
  }
  if (!nh_->has_parameter("cop_threshold"))
    RCLCPP_ERROR_STREAM(nh_->get_logger(), nh_->get_name() << ": cop_threshold not specified");
  cop_threshold_ = nh_->get_parameter("cop_threshold").as_double();

  side_ = side;

  // initialize vector for previous values for average calculations
  for (int i = 0; i < 4; i++) {
    std::vector<double> x;
    for (int j = 0; j < average_; j++) {
      x.push_back(0.0);
    }
    previous_values_.push_back(x);
  }
  current_index_ = 0;

  save_zero_and_scale_values_ = false;
  resetZeroAndScaleValues();

  filtered_pub_ = nh_->create_publisher<bitbots_msgs::msg::FootPressure>(topic + "/filtered", 1);
  cop_pub_ = nh_->create_publisher<geometry_msgs::msg::PointStamped>("/" + cop_lr_, 1);
  std::string wrench_topics[] = {"l_front", "l_back", "r_front", "r_back", "cop"};
  for (int i = 0; i < 5; i++) {
    std::stringstream single_wrench_topic;
    single_wrench_topic << topic << "/wrench/" << wrench_topics[i];
    wrench_pubs_.push_back(nh_->create_publisher<geometry_msgs::msg::WrenchStamped>(single_wrench_topic.str(), 1));
  }
  for (int i = 0; i < 4; i++) {
    std::stringstream single_wrench_frame;
    single_wrench_frame << side << "_"
                        << "cleat_" << wrench_topics[i];
    wrench_frames_.push_back(single_wrench_frame.str());
  }

  scale_service_ = nh_->create_service<bitbots_msgs::srv::FootScale>(
      topic + "/set_foot_scale", std::bind(&PressureConverter::scaleCallback, this, _1, _2));
  zero_service_ = nh_->create_service<std_srvs::srv::Empty>(topic + "/set_foot_zero",
                                                            std::bind(&PressureConverter::zeroCallback, this, _1, _2));
  rclcpp::SubscriptionOptions options;
  rclcpp::QoS qos(0);
  options.callback_group = sub_cbg_;
  sub_ = nh_->create_subscription<bitbots_msgs::msg::FootPressure>(
      topic + "/raw", qos, std::bind(&PressureConverter::pressureCallback, this, _1), options);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*nh_);

  sub_executor_.add_callback_group(sub_cbg_, nh_->get_node_base_interface());
  sub_executor_thread_ = new std::thread([this]() { sub_executor_.spin(); });
}

void PressureConverter::pressureCallback(bitbots_msgs::msg::FootPressure pressure_raw) {
  bitbots_msgs::msg::FootPressure filtered_msg;

  filtered_msg.header = pressure_raw.header;

  previous_values_[0][current_index_] = ((pressure_raw.left_front) - zero_[0]) * scale_[0];
  previous_values_[1][current_index_] = ((pressure_raw.left_back) - zero_[1]) * scale_[1],
  previous_values_[2][current_index_] = ((pressure_raw.right_front) - zero_[2]) * scale_[2],
  previous_values_[3][current_index_] = ((pressure_raw.right_back) - zero_[3]) * scale_[3];

  filtered_msg.left_front = std::accumulate(previous_values_[0].begin(), previous_values_[0].end(), 0.0) / average_;
  filtered_msg.left_back = std::accumulate(previous_values_[1].begin(), previous_values_[1].end(), 0.0) / average_;
  filtered_msg.right_front = std::accumulate(previous_values_[2].begin(), previous_values_[2].end(), 0.0) / average_;
  filtered_msg.right_back = std::accumulate(previous_values_[3].begin(), previous_values_[3].end(), 0.0) / average_;
  current_index_ = (current_index_ + 1) % average_;

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
  if (save_zero_and_scale_values_) {
    zero_and_scale_values_[0].push_back(pressure_raw.left_front);
    zero_and_scale_values_[1].push_back(pressure_raw.left_back);
    zero_and_scale_values_[2].push_back(pressure_raw.right_front);
    zero_and_scale_values_[3].push_back(pressure_raw.right_back);
  }

  // publish center of pressure
  double pos_x = 0.085, pos_y = 0.045;
  geometry_msgs::msg::PointStamped cop;
  cop.header.frame_id = sole_lr_;
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
  cop_tf.child_frame_id = cop_lr_;
  cop_tf.transform.translation.x = cop.point.x;
  cop_tf.transform.translation.y = cop.point.y;
  cop_tf.transform.rotation.w = 1;
  tf_broadcaster_->sendTransform(cop_tf);

  geometry_msgs::msg::WrenchStamped w_cop;
  w_cop.header.frame_id = cop_lr_;
  w_cop.header.stamp = pressure_raw.header.stamp;
  w_cop.wrench.force.z = sum_of_forces;
  wrench_pubs_[4]->publish(w_cop);
}

void PressureConverter::resetZeroAndScaleValues() {
  zero_and_scale_values_.clear();
  for (int i = 0; i < 4; i++) {
    std::vector<double> x;
    zero_and_scale_values_.push_back(x);
  }
}

void PressureConverter::collectMessages() {
  save_zero_and_scale_values_ = true;
  while (int(zero_and_scale_values_[0].size()) < scale_and_zero_average_) {
    RCLCPP_WARN_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 0.25, "%ld of %d msgs", zero_and_scale_values_[0].size(),
                         scale_and_zero_average_);
  }
  save_zero_and_scale_values_ = false;
}

void PressureConverter::saveYAML() {
  std::string path_to_yaml, robot_name, command;
  if (std::getenv("ROBOT_NAME")) {
    robot_name = std::getenv("ROBOT_NAME");
  } else {
    robot_name = "nobot";
  }
  path_to_yaml =
      ament_index_cpp::get_package_share_directory("bitbots_ros_control") + "/config/pressure_" + robot_name + ".yaml";

  std::vector<double> scale_r, scale_l, zero_r, zero_l;
  zero_r = nh_->get_parameter("zero_r").as_double_array();
  zero_l = nh_->get_parameter("zero_l").as_double_array();
  scale_r = nh_->get_parameter("scale_r").as_double_array();
  scale_l = nh_->get_parameter("scale_l").as_double_array();

  YAML::Emitter e;
  e << YAML::BeginMap;
  e << YAML::Key << "pressure_converter";
  e << YAML::BeginMap;
  e << YAML::Key << "ros__parameters";
  e << YAML::BeginMap;
  e << YAML::Key << "zero_r";
  e << YAML::Value << zero_r;
  e << YAML::Key << "zero_l";
  e << YAML::Value << zero_l;
  e << YAML::Key << "scale_r";
  e << YAML::Value << scale_r;
  e << YAML::Key << "scale_l";
  e << YAML::Value << scale_r;
  e << YAML::EndMap;
  e << YAML::EndMap;

  std::ofstream yaml_file;
  yaml_file.open(path_to_yaml);
  yaml_file << e.c_str();
  yaml_file.close();
}

bool PressureConverter::scaleCallback(const std::shared_ptr<bitbots_msgs::srv::FootScale::Request> req,
                                      std::shared_ptr<bitbots_msgs::srv::FootScale::Response> resp) {
  collectMessages();
  double average =
      std::accumulate(zero_and_scale_values_[req->sensor].begin(), zero_and_scale_values_[req->sensor].end(), 0.0) /
      zero_and_scale_values_[req->sensor].size();
  RCLCPP_WARN_STREAM(nh_->get_logger(), "avg: " << average);
  average -= zero_[req->sensor];
  RCLCPP_WARN_STREAM(nh_->get_logger(), "avg_after: " << average);

  scale_[req->sensor] = req->weight / average;
  resetZeroAndScaleValues();
  nh_->set_parameter(rclcpp::Parameter(scale_lr_, rclcpp::ParameterValue(scale_)));
  saveYAML();
  return true;
}

bool PressureConverter::zeroCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                     std::shared_ptr<std_srvs::srv::Empty::Response> resp) {
  collectMessages();
  for (int i = 0; i < 4; i++) {
    zero_[i] = std::accumulate(zero_and_scale_values_[i].begin(), zero_and_scale_values_[i].end(), 0.0) /
               zero_and_scale_values_[i].size();
  }
  resetZeroAndScaleValues();
  nh_->set_parameter(rclcpp::Parameter(zero_lr_, rclcpp::ParameterValue(zero_)));
  saveYAML();
  return true;
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // declare parameters automatically
  rclcpp::NodeOptions options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("pressure_converter", options);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(nh);

  PressureConverter r(nh, 'r');
  PressureConverter l(nh, 'l');

  executor.spin();

  return 0;
}
