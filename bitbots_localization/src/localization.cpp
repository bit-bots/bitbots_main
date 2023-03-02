//
// Created by judith on 08.03.19.
//

#include "bitbots_localization/localization.h"

#include <chrono>
#include <thread>


namespace bitbots_localization {

Localization::Localization() :
  Node("bitbots_localization", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)),
  tfBuffer(std::make_unique<tf2_ros::Buffer>(this->get_clock())),
  tfListener(std::make_shared<tf2_ros::TransformListener>(*tfBuffer, this)),
  br(std::make_shared<tf2_ros::TransformBroadcaster>(this)){

  // Wait for transforms to become available and init them
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  while(true){
    try{
      previousOdomTransform_ = tfBuffer->lookupTransform(odom_frame_, base_footprint_frame_, rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(1e9*20.0));
      break;
    }catch (const tf2::LookupException &ex) {
      RCLCPP_INFO(this->get_logger(),"Transforms not available, waiting for them... \n %s", ex.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  RCLCPP_INFO(this->get_logger(),"Transforms are available now");

  auto parameters = this->get_parameters(this->list_parameters({}, 10).names);

  config_ = std::make_shared<Config>(parameters);

  Localization::onSetParameters(parameters);

  param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&Localization::onSetParameters, this, _1));
}

rcl_interfaces::msg::SetParametersResult Localization::onSetParameters(const std::vector<rclcpp::Parameter> &parameters) {

  config_->update_params(parameters);

  odom_frame_ = this->get_parameter("odom_frame").as_string();
  base_footprint_frame_ = this->get_parameter("base_footprint_frame").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  publishing_frame_ = this->get_parameter("publishing_frame").as_string();

  line_point_cloud_subscriber_ = this->create_subscription<sm::msg::PointCloud2>(
    config_->line_pointcloud_topic, 1, std::bind(&Localization::LinePointcloudCallback, this, _1));
  goal_subscriber_ = this->create_subscription<sv3dm::msg::GoalpostArray>(
    config_->goal_topic, 1, std::bind(&Localization::GoalPostsCallback, this, _1));
  fieldboundary_subscriber_ = this->create_subscription<sv3dm::msg::FieldBoundary>(
    config_->fieldboundary_topic, 1, std::bind(&Localization::FieldboundaryCallback, this, _1));

  pose_particles_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(config_->particle_publishing_topic, 1);

  pose_with_covariance_publisher_ = this->create_publisher<gm::msg::PoseWithCovarianceStamped>("pose_with_covariance", 1);
  lines_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("lines", 1);
  line_ratings_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("line_ratings", 1);
  goal_ratings_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("goal_ratings", 1);
  fieldboundary_ratings_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("field_boundary_ratings", 1);

  // Get field name
  auto field = this->get_parameter("fieldname").as_string();

  // Check if mesurement type is used and load the correct map for that
  if(config_->lines_factor) {
    lines_.reset(new Map(field, "lines.png", -10.0));  // TODO real parameter
  }
  if(config_->goals_factor){
    goals_.reset(new Map(field, "goals.png", -10.0));
  }
  if(config_->field_boundary_factor){
    field_boundary_.reset(new Map(field, "field_boundary.png", -10.0));
  }

  line_pointcloud_relative_.header.stamp = rclcpp::Time(0);
  goal_posts_relative_.header.stamp = rclcpp::Time(0);
  fieldboundary_relative_.header.stamp = rclcpp::Time(0);

  robot_pose_observation_model_.reset(
      new RobotPoseObservationModel(
        lines_, goals_, field_boundary_, config_));
  robot_pose_observation_model_->set_min_weight(config_->min_weight);

  Eigen::Matrix<double, 3, 2> drift_cov;
  drift_cov <<
    // Standard dev of applied drift related to
    // distance, rotation
    config_->drift_distance_to_direction, config_->drift_roation_to_direction,
    config_->drift_distance_to_distance,  config_->drift_roation_to_distance,
    config_->drift_distance_to_rotation,  config_->drift_rotation_to_rotation;

  // Scale drift form drift per second to drift per filter iteration
  drift_cov /= config_->publishing_frequency;

  drift_cov.col(0) *= (1 / (config_->max_translation / config_->publishing_frequency));
  drift_cov.col(1) *= (1 / (config_->max_rotation / config_->publishing_frequency));

  robot_motion_model_.reset(
      new RobotMotionModel(random_number_generator_,
                           config_->diffusion_x_std_dev,
                           config_->diffusion_y_std_dev,
                           config_->diffusion_t_std_dev,
                           config_->starting_diffusion,
                           drift_cov));

  robot_state_distribution_start_left_.reset(
    new RobotStateDistributionStartLeft(
      random_number_generator_,
      std::make_pair(
        config_->field_x,
        config_->field_y)));

  robot_state_distribution_start_right_.reset(
    new RobotStateDistributionStartRight(
      random_number_generator_,
      std::make_pair(
        config_->field_x,
        config_->field_y)));

  robot_state_distribution_left_half_.reset(
    new RobotStateDistributionLeftHalf(
      random_number_generator_,
      std::make_pair(
        config_->field_x,
        config_->field_y)));

  robot_state_distribution_right_half_.reset(
    new RobotStateDistributionRightHalf(
      random_number_generator_,
      std::make_pair(
        config_->field_x,
        config_->field_y)));

  robot_state_distribution_position_.reset(
    new RobotStateDistributionPosition(
      random_number_generator_,
      config_->initial_robot_x,
      config_->initial_robot_y));

  robot_state_distribution_pose_.reset(
    new RobotStateDistributionPose(
      random_number_generator_,
      config_->initial_robot_x,
      config_->initial_robot_y,
      config_->initial_robot_t));

  resampling_.reset(new pf::ImportanceResampling<RobotState>(true, config_->min_resampling_weight));

  RCLCPP_INFO(this->get_logger(),"Trying to initialize particle filter...");
  reset_filter(config_->init_mode);

  if (first_configuration_) {
    first_configuration_ = false;
    reset_service_ = this->create_service<bl::srv::ResetFilter>("reset_localization", std::bind(&Localization::reset_filter_callback, this, _1, _2));
    pause_service_ = this->create_service<bl::srv::SetPaused>("pause_localization", std::bind(&Localization::set_paused_callback, this, _1, _2));
  }

  publishing_timer_ = rclcpp::create_timer(
    this, this->get_clock(),
    rclcpp::Duration(0, uint32_t (1.0e9 / config_->publishing_frequency)),
    std::bind(&Localization::run_filter_one_step, this));

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  return result;
}

void Localization::run_filter_one_step() {
  timer_callback_count_++;
  resampled_ = false;

  // Set the measurements in the observation model
  updateMeasurements();

  // Get the odometry offset since the last cycle
  getMotion();

  // Drops the diffusion noise back to normal if it was bumped by a reset/init.
  // Increasing the noise helps with the initial localization.
  if (timer_callback_count_ > config_->starting_steps_with_higher_diffusion) {
    robot_motion_model_->diffuse_multiplicator_ = config_->diffusion_multiplicator;
  }

  if ((config_->filter_only_with_motion and robot_moved) or (!config_->filter_only_with_motion)) {
    robot_pf_->drift(linear_movement_, rotational_movement_);
    robot_pf_->diffuse();
  }

  // Apply ratings corresponding to the observations compared with each partice position
  robot_pf_->measure();

  // Check if its resampling time!
  if (timer_callback_count_ % config_->resampling_interval == 0) {
    robot_pf_->resample();
    resampled_ = true;
  }
  // Publish transforms
  publish_transforms();
  // Publish covariance message
  publish_pose_with_covariance();
  //Publish debug stuff
  if(config_->debug_visualization) {
    publish_debug();
  }

  robot_pose_observation_model_->clear_measurement();
}

void Localization::LinePointcloudCallback(const sm::msg::PointCloud2 &msg) {
  line_pointcloud_relative_ = msg;
}

void Localization::GoalPostsCallback(const sv3dm::msg::GoalpostArray &msg) {
  goal_posts_relative_ = msg;
}

void Localization::FieldboundaryCallback(const sv3dm::msg::FieldBoundary &msg) {
  fieldboundary_relative_ = msg;
}

bool Localization::set_paused_callback(const std::shared_ptr<bl::srv::SetPaused::Request> req,
                                       std::shared_ptr<bl::srv::SetPaused::Response> res) {
  if(req->paused) {
    publishing_timer_->cancel();
  } else {
    publishing_timer_->reset();
  }
  res->success = true;
  return true;
}

bool Localization::reset_filter_callback(const std::shared_ptr<bl::srv::ResetFilter::Request> req,
                                         std::shared_ptr<bl::srv::ResetFilter::Response> res) {
  if (req->init_mode == 3) {
    reset_filter(req->init_mode, req->x, req->y);
  } else if (req->init_mode == 4) {
    reset_filter(req->init_mode, req->x, req->y, req->angle);
  } else {
    reset_filter(req->init_mode);
  }
  res->success = true;
  return true;
}

void Localization::reset_filter(int distribution) {
  RCLCPP_INFO(this->get_logger(),"reset filter");

  robot_pf_.reset(new particle_filter::ParticleFilter<RobotState>(
      config_->particle_number, robot_pose_observation_model_, robot_motion_model_));

  timer_callback_count_ = 0;

  // Increasing the noise helps with the initial localization.
  robot_motion_model_->diffuse_multiplicator_ = config_->starting_diffusion;

  robot_pf_->setResamplingStrategy(resampling_);
  if (distribution == 0) {
    robot_pf_->drawAllFromDistribution(robot_state_distribution_start_right_);
  } else if (distribution == 1) {
    robot_pf_->drawAllFromDistribution(robot_state_distribution_left_half_);
  } else if (distribution == 2) {
    robot_pf_->drawAllFromDistribution(robot_state_distribution_right_half_);
  } else if (distribution == 4) {
    robot_pf_->drawAllFromDistribution(robot_state_distribution_pose_);
  } else {
    return;
  }
}

void Localization::reset_filter(int distribution, double x, double y) {

  robot_pf_.reset(new particle_filter::ParticleFilter<RobotState>(
      config_->particle_number, robot_pose_observation_model_, robot_motion_model_));

  robot_pf_->setResamplingStrategy(resampling_);

  if (distribution == 3) {
    robot_state_distribution_position_.reset(new RobotStateDistributionPosition(random_number_generator_, x, y));
    robot_pf_->drawAllFromDistribution(robot_state_distribution_position_);
  }
}

void Localization::reset_filter(int distribution, double x, double y, double angle) {

  robot_pf_.reset(new particle_filter::ParticleFilter<RobotState>(
      config_->particle_number, robot_pose_observation_model_, robot_motion_model_));

  robot_pf_->setResamplingStrategy(resampling_);

  if (distribution == 4) {
    robot_state_distribution_pose_.reset(new RobotStateDistributionPose(random_number_generator_, x, y, angle));
    robot_pf_->drawAllFromDistribution(robot_state_distribution_pose_);
  }
}

void Localization::updateMeasurements() {
  // Sets the measurements in the observation model
  if (line_pointcloud_relative_.header.stamp != last_stamp_lines_pc && config_->lines_factor) {
    robot_pose_observation_model_->set_measurement_lines_pc(line_pointcloud_relative_);
  }
  if (config_->goals_factor && goal_posts_relative_.header.stamp != last_stamp_goals) {
    robot_pose_observation_model_->set_measurement_goalposts(goal_posts_relative_);
  }
  if (config_->field_boundary_factor && fieldboundary_relative_.header.stamp != last_stamp_fb_points) {
    robot_pose_observation_model_->set_measurement_field_boundary(fieldboundary_relative_);
  }

  // Set timestamps to mark past messages
  last_stamp_lines_pc = line_pointcloud_relative_.header.stamp;
  last_stamp_goals = goal_posts_relative_.header.stamp;
  last_stamp_fb_points = fieldboundary_relative_.header.stamp;
  // Maximum time stamp of the last measurements
  last_stamp_all_measurments = std::max({last_stamp_lines_pc, last_stamp_goals, last_stamp_fb_points});
}

void Localization::getMotion() {
  robot_moved = false;
  geometry_msgs::msg::TransformStamped transformStampedNow;

  try {

    // Get current odometry transform
    transformStampedNow = tfBuffer->lookupTransform(odom_frame_, base_footprint_frame_, rclcpp::Time(0));

    // Get linear movement from odometry transform and the transform of the previous filter step
    double global_diff_x, global_diff_y;
    global_diff_x = (transformStampedNow.transform.translation.x - previousOdomTransform_.transform.translation.x);
    global_diff_y = (transformStampedNow.transform.translation.y - previousOdomTransform_.transform.translation.y);

    // Convert to local frame
    auto [polar_rot, polar_dist] = cartesianToPolar(global_diff_x, global_diff_y);
    auto [local_movement_x, local_movement_y] = polarToCartesian(
      polar_rot - tf2::getYaw(previousOdomTransform_.transform.rotation), polar_dist);
    linear_movement_.x = local_movement_x;
    linear_movement_.y = local_movement_y;
    linear_movement_.z = 0;

    // Get angular movement from odometry transform and the transform of the previous filter step
    rotational_movement_.x = 0;
    rotational_movement_.y = 0;
    rotational_movement_.z = tf2::getYaw(transformStampedNow.transform.rotation) -
        tf2::getYaw(previousOdomTransform_.transform.rotation);

    // Check if robot moved
    if (linear_movement_.x > config_->min_motion_linear or linear_movement_.y > config_->min_motion_linear or
        rotational_movement_.z > config_->min_motion_angular) {
      robot_moved = true;
    }

    // Set the variable for the transform of the previous step to the transform of the current step, because we finished this step.
    previousOdomTransform_ = transformStampedNow;
  }
  catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),"Could not aquire motion for odom transforms: %s", ex.what());
  }
}

void Localization::publish_transforms() {

  //get estimate and covariance
  auto estimate = robot_pf_->getBestXPercentEstimate(config_->percentage_best_particles);

  // Convert to tf2
  tf2::Transform filter_transform;
  filter_transform.setOrigin(tf2::Vector3(estimate.getXPos() , estimate.getYPos(), 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, estimate.getTheta());
  filter_transform.setRotation(q);

  // Get the transform from the last measurement timestamp until now
  geometry_msgs::msg::TransformStamped odomDuringMeasurement, odomNow;
  try {
   odomDuringMeasurement = tfBuffer->lookupTransform(odom_frame_, base_footprint_frame_, last_stamp_all_measurments);
   odomNow = tfBuffer->lookupTransform(odom_frame_, base_footprint_frame_, rclcpp::Time(0));
  }
  catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),"Could not aquire odom transforms: %s", ex.what());
  }

  // Calculate difference between the two transforms
  tf2::Transform odomDuringMeasurement_tf2, odomNow_tf2;
  tf2::fromMsg(odomDuringMeasurement.transform, odomDuringMeasurement_tf2);
  tf2::fromMsg(odomNow.transform, odomNow_tf2);
  tf2::Transform odom_diff = odomNow_tf2 * odomDuringMeasurement_tf2.inverse();

  // Apply the transform from the last measurement timestamp until now to the current estimate
  filter_transform = odom_diff * filter_transform;

  // Update estimate_ with the new estimate
  estimate_.setXPos(filter_transform.getOrigin().x());
  estimate_.setYPos(filter_transform.getOrigin().y());
  estimate_.setTheta(tf2::getYaw(filter_transform.getRotation()));
  estimate_cov_ = robot_pf_->getCovariance(config_->percentage_best_particles);

  //////////////////////
  //publish transforms//
  //////////////////////

  try{
    // Publish localization tf, not the odom offset
    geometry_msgs::msg::TransformStamped localization_transform;
    localization_transform.header.stamp = odomNow.header.stamp;
    localization_transform.header.frame_id = map_frame_;
    localization_transform.child_frame_id = publishing_frame_;
    localization_transform.transform.translation.x = estimate_.getXPos();
    localization_transform.transform.translation.y = estimate_.getYPos();
    localization_transform.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, estimate_.getTheta());
    q.normalize();
    localization_transform.transform.rotation.x = q.x();
    localization_transform.transform.rotation.y = q.y();
    localization_transform.transform.rotation.z = q.z();
    localization_transform.transform.rotation.w = q.w();

    if (localization_tf_last_published_time_ != localization_transform.header.stamp) {
      // Do not resend a transform for the same timestamp
      localization_tf_last_published_time_ = localization_transform.header.stamp;
      br->sendTransform(localization_transform);
    }

    // Publish odom localisation offset
    geometry_msgs::msg::TransformStamped map_odom_transform;

    map_odom_transform.header.stamp = this->get_clock()->now();
    map_odom_transform.header.frame_id = map_frame_;
    map_odom_transform.child_frame_id = odom_frame_;

    // Calculate odom offset
    tf2::Transform odom_transform_tf, localization_transform_tf, map_tf;
    tf2::fromMsg(odomNow.transform, odom_transform_tf);
    tf2::fromMsg(localization_transform.transform, localization_transform_tf);
    map_tf = localization_transform_tf * odom_transform_tf.inverse();

    map_odom_transform.transform = tf2::toMsg(map_tf);

    RCLCPP_DEBUG(this->get_logger(),"Transform %s", geometry_msgs::msg::to_yaml(map_odom_transform).c_str());

    if (map_odom_tf_last_published_time_ != map_odom_transform.header.stamp) {
       // Do not resend a transform for the same timestamp
      map_odom_tf_last_published_time_ = map_odom_transform.header.stamp;
      br->sendTransform(map_odom_transform);
    }
  }
  catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),"Odom not available, therefore odom offset can not be published: %s", ex.what());
  }
}

void Localization::publish_pose_with_covariance() {
  // Calculate quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, estimate_.getTheta());
  q.normalize();

  gm::msg::PoseWithCovarianceStamped estimateMsg;
  estimateMsg.header.stamp = this->get_clock()->now();
  estimateMsg.pose.pose.orientation.w = q.w();
  estimateMsg.pose.pose.orientation.x = q.x();
  estimateMsg.pose.pose.orientation.y = q.y();
  estimateMsg.pose.pose.orientation.z = q.z();
  estimateMsg.pose.pose.position.x = estimate_.getXPos();
  estimateMsg.pose.pose.position.y = estimate_.getYPos();

  std::vector<double> covariance;

  for (int i = 0; i < 36; i++) {
    estimateMsg.pose.covariance[i] = estimate_cov_[i];
  }

  estimateMsg.header.frame_id = map_frame_;

  pose_with_covariance_publisher_->publish(estimateMsg);
}

void Localization::publish_debug() {
  // Show a marker for each partikle
  publish_particle_markers();
  // Show ratings for each used class
  publish_ratings();
}

void Localization::publish_particle_markers() {
  //publish particle markers
  std_msgs::msg::ColorRGBA red;
  red.r = 1;
  red.g = 0;
  red.b = 0;
  red.a = 1;

  pose_particles_publisher_->publish(
    robot_pf_->renderMarkerArray(
      "pose_marker",
      map_frame_,
      rclcpp::Duration::from_nanoseconds(1e9),
      red,
      this->get_clock()->now()));
}

void Localization::publish_ratings() {
  if (config_->lines_factor) {
    // Publish line ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_lines(), 0.1, "line_ratings", lines_, line_ratings_publisher_);
  }
  if (config_->goals_factor) {
    // Publish goal ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_goals(), 0.2, "goal_ratings", goals_, goal_ratings_publisher_);
  }
  if (config_->field_boundary_factor) {
    // Publish field boundary ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_field_boundary(), 0.2, "field_boundary_ratings", field_boundary_, fieldboundary_ratings_publisher_);
  }
}

void Localization::publish_debug_rating(
    std::vector<std::pair<double, double>> measurements,
    double scale,
    const char name[],
    std::shared_ptr<Map> map,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &publisher) {

  RobotState best_estimate = robot_pf_->getBestXPercentEstimate(config_->percentage_best_particles);
  double rating = 0;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = map_frame_;
  marker.header.stamp = this->get_clock()->now();
  marker.ns = name;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.scale.x = scale;
  marker.scale.y = scale;

  for (std::pair<double, double> &measurement : measurements) {
    //lines are in polar form!
    std::pair<double, double> observationRelative;

    observationRelative = map->observationRelative(
      measurement, best_estimate.getXPos(), best_estimate.getYPos(), best_estimate.getTheta());
    double occupancy = map->get_occupancy(observationRelative.first, observationRelative.second);

    geometry_msgs::msg::Point point;
    point.x = observationRelative.first;
    point.y = observationRelative.second;
    point.z = 0;

    std_msgs::msg::ColorRGBA color;
    color.b = 1;
    if (occupancy >= 0) {
      color.r = occupancy / 100;
    } else {
      color.g = 1;
    }
    color.a = 1;
    marker.points.push_back(point);
    marker.colors.push_back(color);
  }
  publisher->publish(marker);
}
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bitbots_localization::Localization>());
  rclcpp::shutdown();
  return 0;
}
