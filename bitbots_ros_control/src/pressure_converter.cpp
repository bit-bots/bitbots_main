#include <bitbots_ros_control/pressure_converter.h>

PressureConverter::PressureConverter(ros::NodeHandle &pnh, char side) {
  std::string topic;
  if (side == 'l') {
    if (!pnh.getParam("left_topic", topic))
      ROS_ERROR_STREAM(ros::this_node::getName() << ": left_topic not specified");
  } else if (side == 'r') {
    if (!pnh.getParam("right_topic", topic))
      ROS_ERROR_STREAM(ros::this_node::getName() << ": right_topic not specified");
  }

  if (!pnh.getParam("average", average_))
    ROS_ERROR_STREAM(ros::this_node::getName() << ": average not specified");

  if (!pnh.getParam("scale_and_zero_average", scale_and_zero_average_))
    ROS_ERROR_STREAM(ros::this_node::getName() << ": scale_and_zero_average not specified");

  scale_lr_ = "scale_";
  scale_lr_ += side;
  zero_lr_ = "zero_";
  zero_lr_ += side;
  cop_lr_ = "cop_";
  cop_lr_ += side;
  sole_lr_ = side;
  sole_lr_ += "_sole";

  if (!pnh.getParam(zero_lr_, zero_)) {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": " << zero_lr_ << " not specified");
    zero_ = {0, 0, 0, 0};
  }
  if (!pnh.getParam(scale_lr_, scale_)) {
    ROS_ERROR_STREAM(ros::this_node::getName() << ": " << scale_lr_ << " not specified");
    scale_ = {1, 1, 1, 1};
  }
  if (!pnh.getParam("cop_threshold", cop_threshold_))
    ROS_ERROR_STREAM(ros::this_node::getName() << ": cop_threshold not specified");

  side_ = side;
  pnh_ = pnh;

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

  filtered_pub_ = pnh_.advertise<bitbots_msgs::FootPressure>(topic + "/filtered", 1);
  cop_pub_ = pnh_.advertise<geometry_msgs::PointStamped>("/" + cop_lr_, 1);
  std::string wrench_topics[] = {"l_front", "l_back", "r_front", "r_back", "cop"};
  for (int i = 0; i < 5; i++) {
    std::stringstream single_wrench_topic;
    single_wrench_topic << topic << "/wrench/" << wrench_topics[i];
    wrench_pubs_.push_back(pnh_.advertise<geometry_msgs::WrenchStamped>(single_wrench_topic.str(), 1));
  }
  for (int i = 0; i < 4; i++) {
    std::stringstream single_wrench_frame;
    single_wrench_frame << side << "_" << "cleat_" << wrench_topics[i];
    wrench_frames_.push_back(single_wrench_frame.str());
  }
  scale_service_ = pnh_.advertiseService(topic + "/set_foot_scale", &PressureConverter::scaleCallback, this);
  zero_service_ = pnh_.advertiseService(topic + "/set_foot_zero", &PressureConverter::zeroCallback, this);

  sub_ = pnh_.subscribe(topic + "/raw",
                        1,
                        &PressureConverter::pressureCallback,
                        this, ros::TransportHints().tcpNoDelay());
}

void PressureConverter::pressureCallback(const bitbots_msgs::FootPressureConstPtr &pressure_raw) {
  bitbots_msgs::FootPressure filtered_msg;

  filtered_msg.header = pressure_raw->header;

  previous_values_[0][current_index_] = ((pressure_raw->left_front) - zero_[0]) * scale_[0];
  previous_values_[1][current_index_] = ((pressure_raw->left_back) - zero_[1]) * scale_[1],
  previous_values_[2][current_index_] = ((pressure_raw->right_front) - zero_[2]) * scale_[2],
  previous_values_[3][current_index_] = ((pressure_raw->right_back) - zero_[3]) * scale_[3];

  filtered_msg.left_front = std::accumulate(previous_values_[0].begin(), previous_values_[0].end(), 0.0) / average_;
  filtered_msg.left_back = std::accumulate(previous_values_[1].begin(), previous_values_[1].end(), 0.0) / average_;
  filtered_msg.right_front = std::accumulate(previous_values_[2].begin(), previous_values_[2].end(), 0.0) / average_;
  filtered_msg.right_back = std::accumulate(previous_values_[3].begin(), previous_values_[3].end(), 0.0) / average_;
  current_index_ = (current_index_ + 1) % average_;

  filtered_msg.left_front = std::max(filtered_msg.left_front, 0.0);
  filtered_msg.left_back = std::max(filtered_msg.left_back, 0.0);
  filtered_msg.right_front = std::max(filtered_msg.right_front, 0.0);
  filtered_msg.right_back = std::max(filtered_msg.right_back, 0.0);

  std::vector<double> forces_list =
      {filtered_msg.left_front, filtered_msg.left_back, filtered_msg.right_front, filtered_msg.right_back};
  for (int i = 0; i < 4; i++) {
    geometry_msgs::WrenchStamped w;
    w.header.frame_id = wrench_frames_[i];
    w.header.stamp = pressure_raw->header.stamp;
    w.wrench.force.z = forces_list[i];
    wrench_pubs_[i].publish(w);
  }
  filtered_pub_.publish(filtered_msg);
  if (save_zero_and_scale_values_) {
    zero_and_scale_values_[0].push_back(pressure_raw->left_front);
    zero_and_scale_values_[1].push_back(pressure_raw->left_back);
    zero_and_scale_values_[2].push_back(pressure_raw->right_front);
    zero_and_scale_values_[3].push_back(pressure_raw->right_back);
  }

  // publish center of pressure
  double pos_x = 0.085, pos_y = 0.045;
  geometry_msgs::PointStamped cop;
  cop.header.frame_id = sole_lr_;
  cop.header.stamp = pressure_raw->header.stamp;
  double sum_of_forces =
      filtered_msg.left_front + filtered_msg.left_back + filtered_msg.right_back + filtered_msg.right_front;
  if (sum_of_forces > cop_threshold_) {
    cop.point.x =
        (filtered_msg.left_front + filtered_msg.right_front - filtered_msg.left_back - filtered_msg.right_back) * pos_x
            / sum_of_forces;
    cop.point.x = std::max(std::min(cop.point.x, pos_x), -pos_x);

    cop.point.y =
        (filtered_msg.left_front + filtered_msg.left_back - filtered_msg.right_front - filtered_msg.right_back) * pos_y
            / sum_of_forces;
    cop.point.y = std::max(std::min(cop.point.y, pos_y), -pos_y);
  } else {
    cop.point.x = 0;
    cop.point.y = 0;
  }
  cop_pub_.publish(cop);

  geometry_msgs::TransformStamped cop_tf;
  cop_tf.header = cop.header;
  cop_tf.child_frame_id = cop_lr_;
  cop_tf.transform.translation.x = cop.point.x;
  cop_tf.transform.translation.y = cop.point.y;
  cop_tf.transform.rotation.w = 1;
  tf_broadcaster_.sendTransform(cop_tf);

  geometry_msgs::WrenchStamped w_cop;
  w_cop.header.frame_id = cop_lr_;
  w_cop.header.stamp = pressure_raw->header.stamp;
  w_cop.wrench.force.z = sum_of_forces;
  wrench_pubs_[4].publish(w_cop);
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
  while (zero_and_scale_values_[0].size() < scale_and_zero_average_) {
    ROS_WARN_THROTTLE(0.25, "%ld of %d msgs", zero_and_scale_values_[0].size(), scale_and_zero_average_);
    ros::spinOnce();
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
  path_to_yaml = ros::package::getPath("bitbots_ros_control") + "/config/pressure_" + robot_name + ".yaml";

  std::vector<double> scale_r, scale_l, zero_r, zero_l;
  pnh_.getParam("zero_r", zero_r);
  pnh_.getParam("zero_l", zero_l);
  pnh_.getParam("scale_r", scale_r);
  pnh_.getParam("scale_l", scale_l);

  YAML::Emitter e;
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

  std::ofstream yaml_file;
  yaml_file.open(path_to_yaml);
  yaml_file << e.c_str();
  yaml_file.close();
}

bool PressureConverter::scaleCallback(bitbots_msgs::FootScaleRequest &req, bitbots_msgs::FootScaleResponse &resp) {
  collectMessages();
  double average =
      std::accumulate(zero_and_scale_values_[req.sensor].begin(), zero_and_scale_values_[req.sensor].end(), 0.0)
          / zero_and_scale_values_[req.sensor].size();
  ROS_WARN_STREAM("avg: " << average);
  average -= zero_[req.sensor];
  ROS_WARN_STREAM("avg_after: " << average);

  scale_[req.sensor] = req.weight / average;
  resetZeroAndScaleValues();
  pnh_.setParam(scale_lr_, scale_);
  saveYAML();
  return true;
}

bool PressureConverter::zeroCallback(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &resp) {
  collectMessages();
  for (int i = 0; i < 4; i++) {
    zero_[i] = std::accumulate(zero_and_scale_values_[i].begin(), zero_and_scale_values_[i].end(), 0.0)
        / zero_and_scale_values_[i].size();
  }
  resetZeroAndScaleValues();
  pnh_.setParam(zero_lr_, zero_);
  saveYAML();
  return true;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "pressure_converter");
  ros::NodeHandle pnh("~");
  std::string left_topic, right_topic;

  PressureConverter r(pnh, 'r');
  PressureConverter l(pnh, 'l');

  ros::spin();

  return 0;
}
