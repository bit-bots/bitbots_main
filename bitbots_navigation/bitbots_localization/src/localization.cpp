#include "bitbots_localization/localization.hpp"

#include <chrono>
#include <thread>

namespace bitbots_localization {

Localization::Localization(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      param_listener_(node->get_node_parameters_interface()),
      config_(param_listener_.get_params()),
      tfBuffer(std::make_unique<tf2_ros::Buffer>(node->get_clock())),
      tfListener(std::make_shared<tf2_ros::TransformListener>(*tfBuffer, node)),
      br(std::make_shared<tf2_ros::TransformBroadcaster>(node)) {
  // Wait for transforms to become available and init them
  bitbots_utils::wait_for_tf(node->get_logger(), node->get_clock(), tfBuffer.get(), {config_.ros.base_footprint_frame},
                             config_.ros.odom_frame);

  // Get the initial transform from odom to base_footprint
  previousOdomTransform_ = tfBuffer->lookupTransform(config_.ros.odom_frame, config_.ros.base_footprint_frame,
                                                     rclcpp::Time(0), rclcpp::Duration::from_nanoseconds(1e9 * 20.0));

  // Init subscribers
  line_point_cloud_subscriber_ = node->create_subscription<sm::msg::PointCloud2>(
      config_.ros.line_pointcloud_topic, 1, std::bind(&Localization::LinePointcloudCallback, this, _1));

  goal_subscriber_ = node->create_subscription<sv3dm::msg::GoalpostArray>(
      config_.ros.goal_topic, 1, std::bind(&Localization::GoalPostsCallback, this, _1));

  fieldboundary_subscriber_ = node->create_subscription<sv3dm::msg::FieldBoundary>(
      config_.ros.fieldboundary_topic, 1, std::bind(&Localization::FieldboundaryCallback, this, _1));

  rviz_initial_pose_subscriber_ = node->create_subscription<gm::msg::PoseWithCovarianceStamped>(
      "initialpose", 1, std::bind(&Localization::SetInitialPositionCallback, this, _1));

  // Init publishers
  pose_particles_publisher_ =
      node->create_publisher<visualization_msgs::msg::MarkerArray>(config_.ros.particle_publishing_topic, 1);
  pose_with_covariance_publisher_ =
      node->create_publisher<gm::msg::PoseWithCovarianceStamped>("pose_with_covariance", 1);
  lines_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("lines", 1);
  line_ratings_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("line_ratings", 1);
  goal_ratings_publisher_ = node->create_publisher<visualization_msgs::msg::Marker>("goal_ratings", 1);
  fieldboundary_ratings_publisher_ =
      node->create_publisher<visualization_msgs::msg::Marker>("field_boundary_ratings", 1);
  field_publisher_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "field/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // Set the measurement timestamps to 0
  line_pointcloud_relative_.header.stamp = rclcpp::Time(0);
  goal_posts_relative_.header.stamp = rclcpp::Time(0);
  fieldboundary_relative_.header.stamp = rclcpp::Time(0);

  // Get the static global configuration from the blackboard
  auto global_params = bitbots_utils::get_parameters_from_other_node(
      node_, "/parameter_blackboard", {"field.size.x", "field.size.y", "field.size.padding", "field.name"}, 1s);
  field_name_ = global_params["field.name"].as_string();
  field_dimensions_.x = global_params["field.size.x"].as_double();
  field_dimensions_.y = global_params["field.size.y"].as_double();
  field_dimensions_.padding = global_params["field.size.padding"].as_double();

  // Update all things that are dependent on the parameters and
  // might need to be updated during runtime later if a parameter is changed
  updateParams(true);

  // Init the particles with the given distribution
  RCLCPP_INFO(node->get_logger(), "Trying to initialize particle filter...");
  reset_filter(config_.misc.init_mode);

  // Init services that can be called from outside
  reset_service_ = node->create_service<bl::srv::ResetFilter>(
      "reset_localization", std::bind(&Localization::reset_filter_callback, this, _1, _2));
  pause_service_ = node->create_service<bl::srv::SetPaused>(
      "pause_localization", std::bind(&Localization::set_paused_callback, this, _1, _2));

  // Start the timer that runs the filter
  publishing_timer_ = rclcpp::create_timer(node_, node->get_clock(),
                                           rclcpp::Duration(0, uint32_t(1.0e9 / config_.particle_filter.rate)),
                                           std::bind(&Localization::run_filter_one_step, this));
}

void Localization::updateParams(bool force_reload) {
  // Check if we don't need to update the parameters
  if (!force_reload and !param_listener_.is_old(config_)) {
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Updating parameters...");

  // Update parameters
  param_listener_.refresh_dynamic_parameters();
  config_ = param_listener_.get_params();

  // Check if measurement type is used and load the correct map for that
  if (config_.particle_filter.scoring.lines.factor) {
    lines_.reset(new Map(field_name_, "lines.png", config_.particle_filter.scoring.lines.out_of_field_score));
    // Publish the line map once
    field_publisher_->publish(lines_->get_map_msg(config_.ros.map_frame));
  }
  if (config_.particle_filter.scoring.goal.factor) {
    goals_.reset(new Map(field_name_, "goals.png", config_.particle_filter.scoring.goal.out_of_field_score));
  }
  if (config_.particle_filter.scoring.field_boundary.factor) {
    field_boundary_.reset(
        new Map(field_name_, "field_boundary.png", config_.particle_filter.scoring.field_boundary.out_of_field_score));
  }

  // Init observation model
  robot_pose_observation_model_.reset(
      new RobotPoseObservationModel(lines_, goals_, field_boundary_, config_, field_dimensions_));

  // Init motion model
  auto drift_config = config_.particle_filter.drift;
  Eigen::Matrix<double, 3, 2> drift_cov;
  drift_cov <<
      // Standard dev of applied drift related to
      // distance, rotation
      drift_config.distance_to_direction,
      drift_config.rotation_to_direction, drift_config.distance_to_distance, drift_config.rotation_to_distance,
      drift_config.distance_to_rotation, drift_config.rotation_to_rotation;

  // Scale drift form drift per second to drift per filter iteration
  drift_cov /= config_.particle_filter.rate;

  // Scale drift
  drift_cov.col(0) *= (1 / (config_.particle_filter.drift.max_translation / config_.particle_filter.rate));
  drift_cov.col(1) *= (1 / (config_.particle_filter.drift.max_rotation / config_.particle_filter.rate));

  // Create the motion model which updates the particles with the odometry data and adds noise to the different states
  robot_motion_model_.reset(new RobotMotionModel(random_number_generator_, config_.particle_filter.diffusion.x_std_dev,
                                                 config_.particle_filter.diffusion.y_std_dev,
                                                 config_.particle_filter.diffusion.t_std_dev,
                                                 config_.particle_filter.diffusion.starting_multiplier, drift_cov));

  // Create standard particle probability distributions (e.g. for the initialization at the start of the game)
  robot_state_distribution_own_sidelines.reset(new RobotStateDistributionOwnSideline(
      random_number_generator_, std::make_pair(field_dimensions_.x, field_dimensions_.y)));
  robot_state_distribution_opponent_half.reset(new RobotStateDistributionOpponentHalf(
      random_number_generator_, std::make_pair(field_dimensions_.x, field_dimensions_.y)));
  robot_state_distribution_own_half_.reset(new RobotStateDistributionOwnHalf(
      random_number_generator_, std::make_pair(field_dimensions_.x, field_dimensions_.y)));

  // Create the resampling strategy
  resampling_.reset(
      new pf::ImportanceResampling<RobotState>(true, config_.particle_filter.weighting.particle_reset_weight));

  // Check if we need to create a new particle filter or if we can update the existing one (keeping the particle states)
  if (!robot_pf_) {
    // Create new particle filter
    robot_pf_.reset(new particle_filter::ParticleFilter<RobotState>(
        config_.particle_filter.particle_number, robot_pose_observation_model_, robot_motion_model_));
  } else {
    // Update particle filter's components
    robot_pf_->setResamplingStrategy(resampling_);
    robot_pf_->setObservationModel(robot_pose_observation_model_);
    robot_pf_->setMovementModel(robot_motion_model_);
  }
}

void Localization::run_filter_one_step() {
  timer_callback_count_++;

  // Check for new parameters and recreate necessary components if needed
  updateParams();

  // Set the measurements in the observation model
  updateMeasurements();

  // Get the odometry offset since the last cycle
  getMotion();

  // Drops the diffusion noise back to normal if it was bumped by a reset/init.
  // Increasing the noise helps with the initial localization.
  if (timer_callback_count_ > config_.particle_filter.diffusion.starting_steps_with_higher_diffusion) {
    robot_motion_model_->diffuse_multiplier_ = config_.particle_filter.diffusion.multiplier;
  }

  if ((config_.misc.filter_only_with_motion and robot_moved) or (!config_.misc.filter_only_with_motion)) {
    robot_pf_->drift(linear_movement_, rotational_movement_);
    robot_pf_->diffuse();
  }

  // Apply ratings corresponding to the observations compared with each particle position
  robot_pf_->measure();

  // Check if its resampling time!
  if (timer_callback_count_ % config_.particle_filter.resampling_interval == 0) {
    robot_pf_->resample();
  }
  // Publish transforms
  publish_transforms();
  // Publish covariance message
  publish_pose_with_covariance();
  // Publish debug stuff
  if (config_.ros.debug_visualization) {
    publish_debug();
  }

  robot_pose_observation_model_->clear_measurement();
}

void Localization::LinePointcloudCallback(const sm::msg::PointCloud2 &msg) { line_pointcloud_relative_ = msg; }

void Localization::GoalPostsCallback(const sv3dm::msg::GoalpostArray &msg) { goal_posts_relative_ = msg; }

void Localization::FieldboundaryCallback(const sv3dm::msg::FieldBoundary &msg) { fieldboundary_relative_ = msg; }

void Localization::SetInitialPositionCallback(const gm::msg::PoseWithCovarianceStamped &msg) {
  // Transform the given pose to map frame
  auto pose_in_map = tfBuffer->transform(msg, config_.ros.map_frame, tf2::durationFromSec(1.0));

  // Get yaw from quaternion
  double yaw = tf2::getYaw(pose_in_map.pose.pose.orientation);

  // Reset filter
  reset_filter(bl::srv::ResetFilter::Request::POSE, pose_in_map.pose.pose.position.x, pose_in_map.pose.pose.position.y,
               yaw);
}

bool Localization::set_paused_callback(const std::shared_ptr<bl::srv::SetPaused::Request> req,
                                       std::shared_ptr<bl::srv::SetPaused::Response> res) {
  if (req->paused) {
    publishing_timer_->cancel();
  } else {
    publishing_timer_->reset();
  }
  res->success = true;
  return true;
}

bool Localization::reset_filter_callback(const std::shared_ptr<bl::srv::ResetFilter::Request> req,
                                         std::shared_ptr<bl::srv::ResetFilter::Response> res) {
  if (req->init_mode == bl::srv::ResetFilter::Request::POSITION) {
    reset_filter(req->init_mode, req->x, req->y);
  } else if (req->init_mode == bl::srv::ResetFilter::Request::POSE) {
    reset_filter(req->init_mode, req->x, req->y, req->angle);
  } else {
    reset_filter(req->init_mode);
  }
  res->success = true;
  return true;
}

void Localization::reset_filter(int distribution) {
  RCLCPP_INFO(node_->get_logger(), "reset filter");

  robot_pf_.reset(new particle_filter::ParticleFilter<RobotState>(config_.particle_filter.particle_number,
                                                                  robot_pose_observation_model_, robot_motion_model_));

  timer_callback_count_ = 0;

  // Increasing the noise helps with the initial localization.
  robot_motion_model_->diffuse_multiplier_ = config_.particle_filter.diffusion.starting_multiplier;

  robot_pf_->setResamplingStrategy(resampling_);
  if (distribution == 0) {
    robot_pf_->drawAllFromDistribution(robot_state_distribution_own_sidelines);
  } else if (distribution == 1) {
    robot_pf_->drawAllFromDistribution(robot_state_distribution_opponent_half);
  } else if (distribution == 2) {
    robot_pf_->drawAllFromDistribution(robot_state_distribution_own_half_);
  } else {
    RCLCPP_WARN(node_->get_logger(), "Unknown distribution type %d or missing parameters", distribution);
    return;
  }
}

void Localization::reset_filter(int distribution, double x, double y) {
  robot_pf_.reset(new particle_filter::ParticleFilter<RobotState>(config_.particle_filter.particle_number,
                                                                  robot_pose_observation_model_, robot_motion_model_));

  robot_pf_->setResamplingStrategy(resampling_);

  if (distribution == 3) {
    robot_pf_->drawAllFromDistribution(
        std::make_shared<RobotStateDistributionPosition>(random_number_generator_, x, y));
  }
}

void Localization::reset_filter(int distribution, double x, double y, double angle) {
  robot_pf_.reset(new particle_filter::ParticleFilter<RobotState>(config_.particle_filter.particle_number,
                                                                  robot_pose_observation_model_, robot_motion_model_));

  robot_pf_->setResamplingStrategy(resampling_);

  if (distribution == 4) {
    robot_pf_->drawAllFromDistribution(
        std::make_shared<RobotStateDistributionPose>(random_number_generator_, x, y, angle));
  }
}

void Localization::updateMeasurements() {
  // Sets the measurements in the observation model
  if (line_pointcloud_relative_.header.stamp != last_stamp_lines && config_.particle_filter.scoring.lines.factor) {
    robot_pose_observation_model_->set_measurement_lines_pc(line_pointcloud_relative_);
  }
  if (config_.particle_filter.scoring.goal.factor && goal_posts_relative_.header.stamp != last_stamp_goals) {
    robot_pose_observation_model_->set_measurement_goalposts(goal_posts_relative_);
  }
  if (config_.particle_filter.scoring.field_boundary.factor &&
      fieldboundary_relative_.header.stamp != last_stamp_fb_points) {
    robot_pose_observation_model_->set_measurement_field_boundary(fieldboundary_relative_);
  }

  // Set timestamps to mark past messages
  last_stamp_lines = line_pointcloud_relative_.header.stamp;
  last_stamp_goals = goal_posts_relative_.header.stamp;
  last_stamp_fb_points = fieldboundary_relative_.header.stamp;
}

void Localization::getMotion() {
  geometry_msgs::msg::TransformStamped transformStampedNow;

  try {
    // Get current odometry transform
    transformStampedNow =
        tfBuffer->lookupTransform(config_.ros.odom_frame, config_.ros.base_footprint_frame, rclcpp::Time(0));

    // Get linear movement from odometry transform and the transform of the previous filter step
    double global_diff_x, global_diff_y;
    global_diff_x = (transformStampedNow.transform.translation.x - previousOdomTransform_.transform.translation.x);
    global_diff_y = (transformStampedNow.transform.translation.y - previousOdomTransform_.transform.translation.y);

    // Convert to local frame
    auto [polar_rot, polar_dist] = cartesianToPolar(global_diff_x, global_diff_y);
    auto [local_movement_x, local_movement_y] =
        polarToCartesian(polar_rot - tf2::getYaw(previousOdomTransform_.transform.rotation), polar_dist);
    linear_movement_.x = local_movement_x;
    linear_movement_.y = local_movement_y;
    linear_movement_.z = 0;

    // Get angular movement from odometry transform and the transform of the previous filter step
    rotational_movement_.x = 0;
    rotational_movement_.y = 0;
    rotational_movement_.z =
        tf2::getYaw(transformStampedNow.transform.rotation) - tf2::getYaw(previousOdomTransform_.transform.rotation);

    // Get the time delta between the two transforms
    double time_delta = rclcpp::Time(transformStampedNow.header.stamp).seconds() -
                        rclcpp::Time(previousOdomTransform_.header.stamp).seconds();

    // Check if we already applied the motion for this time step
    if (time_delta > 0) {
      // Calculate normalized motion (motion per second)
      auto linear_movement_normalized_x = linear_movement_.x / time_delta;
      auto linear_movement_normalized_y = linear_movement_.y / time_delta;
      auto rotational_movement_normalized_z = rotational_movement_.z / time_delta;

      // Check if the robot moved an unreasonable amount and drop the motion if it did
      if (std::abs(linear_movement_normalized_x) > config_.misc.max_motion_linear or
          std::abs(linear_movement_normalized_y) > config_.misc.max_motion_linear or
          std::abs(std::remainder(rotational_movement_normalized_z, 2 * M_PI)) > config_.misc.max_motion_angular) {
        rotational_movement_.z = 0;
        linear_movement_.x = 0;
        linear_movement_.y = 0;
        RCLCPP_WARN(node_->get_logger(), "Robot moved an unreasonable amount, dropping motion.");
      }

      // Check if robot moved
      robot_moved = linear_movement_normalized_x >= config_.misc.min_motion_linear or
                    linear_movement_normalized_y >= config_.misc.min_motion_linear or
                    rotational_movement_normalized_z >= config_.misc.min_motion_angular;
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Time step delta of zero encountered! Odometry is unavailable.");
      robot_moved = false;
    }

    // Set the variable for the transform of the previous step to the transform of the current step, because we finished
    // this step.
    previousOdomTransform_ = transformStampedNow;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not acquire motion for odom transforms: %s", ex.what());
  }
}

void Localization::publish_transforms() {
  // get estimate and covariance
  auto estimate = robot_pf_->getBestXPercentEstimate(config_.misc.percentage_best_particles);

  // Convert to tf2
  tf2::Transform filter_transform;
  filter_transform.setOrigin(tf2::Vector3(estimate.getXPos(), estimate.getYPos(), 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, estimate.getTheta());
  filter_transform.setRotation(q);

  // Get the transform from the last measurement timestamp until now
  geometry_msgs::msg::TransformStamped odomDuringMeasurement, odomNow;
  try {
    odomNow = tfBuffer->lookupTransform(config_.ros.odom_frame, config_.ros.base_footprint_frame, rclcpp::Time(0));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not acquire odom transforms: %s", ex.what());
  }

  // Update estimate_ with the new estimate
  estimate_.setXPos(filter_transform.getOrigin().x());
  estimate_.setYPos(filter_transform.getOrigin().y());
  estimate_.setTheta(tf2::getYaw(filter_transform.getRotation()));
  estimate_cov_ = robot_pf_->getCovariance(config_.misc.percentage_best_particles);

  //////////////////////
  // publish transforms//
  //////////////////////

  try {
    // Publish localization tf, not the odom offset
    geometry_msgs::msg::TransformStamped localization_transform;
    localization_transform.header.stamp = odomNow.header.stamp;
    localization_transform.header.frame_id = config_.ros.map_frame;
    localization_transform.child_frame_id = config_.ros.publishing_frame;
    localization_transform.transform.translation.x = estimate_.getXPos();
    localization_transform.transform.translation.y = estimate_.getYPos();
    localization_transform.transform.translation.z = 0.0;
    q = tf2::Quaternion();
    q.setRPY(0, 0, estimate_.getTheta());
    q.normalize();
    localization_transform.transform.rotation.x = q.x();
    localization_transform.transform.rotation.y = q.y();
    localization_transform.transform.rotation.z = q.z();
    localization_transform.transform.rotation.w = q.w();

    // Check if a transform for the current timestamp was already published
    if (localization_tf_last_published_time_ != localization_transform.header.stamp) {
      // Do not resend a transform for the same timestamp
      localization_tf_last_published_time_ = localization_transform.header.stamp;
      br->sendTransform(localization_transform);
    }

    // Publish odom localization offset
    geometry_msgs::msg::TransformStamped map_odom_transform;

    map_odom_transform.header.stamp = node_->get_clock()->now();
    map_odom_transform.header.frame_id = config_.ros.map_frame;
    map_odom_transform.child_frame_id = config_.ros.odom_frame;

    // Calculate odom offset
    tf2::Transform odom_transform_tf, localization_transform_tf, map_tf;
    tf2::fromMsg(odomNow.transform, odom_transform_tf);
    tf2::fromMsg(localization_transform.transform, localization_transform_tf);
    map_tf = localization_transform_tf * odom_transform_tf.inverse();

    map_odom_transform.transform = tf2::toMsg(map_tf);

    RCLCPP_DEBUG(node_->get_logger(), "Transform %s", geometry_msgs::msg::to_yaml(map_odom_transform).c_str());

    // Check if a transform for the current timestamp was already published
    if (map_odom_tf_last_published_time_ != map_odom_transform.header.stamp) {
      // Do not resend a transform for the same timestamp
      map_odom_tf_last_published_time_ = map_odom_transform.header.stamp;
      br->sendTransform(map_odom_transform);
    }
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Odom not available, therefore odom offset can not be published: %s", ex.what());
  }
}

void Localization::publish_pose_with_covariance() {
  // Calculate quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, estimate_.getTheta());
  q.normalize();

  gm::msg::PoseWithCovarianceStamped estimateMsg;
  estimateMsg.header.stamp = node_->get_clock()->now();
  estimateMsg.pose.pose.orientation.w = q.w();
  estimateMsg.pose.pose.orientation.x = q.x();
  estimateMsg.pose.pose.orientation.y = q.y();
  estimateMsg.pose.pose.orientation.z = q.z();
  estimateMsg.pose.pose.position.x = estimate_.getXPos();
  estimateMsg.pose.pose.position.y = estimate_.getYPos();

  for (int i = 0; i < 36; i++) {
    estimateMsg.pose.covariance[i] = estimate_cov_[i];
  }

  estimateMsg.header.frame_id = config_.ros.map_frame;

  pose_with_covariance_publisher_->publish(estimateMsg);
}

void Localization::publish_debug() {
  // Show a marker for each particle
  publish_particle_markers();
  // Show ratings for each used class
  publish_ratings();
}

void Localization::publish_particle_markers() {
  // publish particle markers
  std_msgs::msg::ColorRGBA red;
  red.r = 1;
  red.g = 0;
  red.b = 0;
  red.a = 1;

  pose_particles_publisher_->publish(robot_pf_->renderMarkerArray(
      "pose_marker", config_.ros.map_frame, rclcpp::Duration::from_nanoseconds(1e9), red, node_->get_clock()->now()));
}

void Localization::publish_ratings() {
  if (config_.particle_filter.scoring.lines.factor) {
    // Publish line ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_lines(), 0.1, "line_ratings", lines_,
                         line_ratings_publisher_);
  }
  if (config_.particle_filter.scoring.goal.factor) {
    // Publish goal ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_goals(), 0.2, "goal_ratings", goals_,
                         goal_ratings_publisher_);
  }
  if (config_.particle_filter.scoring.field_boundary.factor) {
    // Publish field boundary ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_field_boundary(), 0.2, "field_boundary_ratings",
                         field_boundary_, fieldboundary_ratings_publisher_);
  }
}

void Localization::publish_debug_rating(std::vector<std::pair<double, double>> measurements, double scale,
                                        const char name[], std::shared_ptr<Map> map,
                                        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &publisher) {
  RobotState best_estimate = robot_pf_->getBestXPercentEstimate(config_.misc.percentage_best_particles);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config_.ros.map_frame;
  marker.header.stamp = node_->get_clock()->now();
  marker.ns = name;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.scale.x = scale;
  marker.scale.y = scale;

  for (std::pair<double, double> &measurement : measurements) {
    // lines are in polar form!
    std::pair<double, double> observationRelative;

    observationRelative = map->observationRelative(measurement, best_estimate.getXPos(), best_estimate.getYPos(),
                                                   best_estimate.getTheta());
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
}  // namespace bitbots_localization

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bitbots_localization");
  [[maybe_unused]] auto localization = bitbots_localization::Localization(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
