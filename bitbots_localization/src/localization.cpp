//
// Created by judith on 08.03.19.
//

#include "bitbots_localization/localization.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "localization");
  Localization localization;

  // dynamic reconfigure
  dynamic_reconfigure::Server<bitbots_localization::LocalizationConfig> dynamic_reconfigure_server;
  dynamic_reconfigure::Server<bitbots_localization::LocalizationConfig>::CallbackType f = boost::bind(
      &Localization::dynamic_reconfigure_callback, &localization, _1, _2);
  dynamic_reconfigure_server.setCallback(f); // automatically calls the callback once

  ros::spin();
  return 0;
}

Localization::Localization() : line_points_(), tfListener(tfBuffer) {
  ROS_DEBUG("localization");
  nh_ = ros::NodeHandle("/bitbots_localization");
}

void Localization::dynamic_reconfigure_callback(bl::LocalizationConfig &config, uint32_t config_level) {
  line_subscriber_ = nh_.subscribe(config.line_topic, 1, &Localization::LineCallback, this);
  goal_subscriber_ = nh_.subscribe(config.goal_topic, 1, &Localization::GoalCallback, this);
  fieldboundary_subscriber_ = nh_.subscribe(config.fieldboundary_topic, 1, &Localization::FieldboundaryCallback,
                                            this);
  corners_subscriber_ = nh_.subscribe(config.corners_topic, 1, &Localization::CornerCallback, this);
  t_crossings_subscriber_ = nh_.subscribe(config.tcrossings_topic, 1, &Localization::TCrossingsCallback, this);
  crosses_subscriber_ = nh_.subscribe(config.crosses_topic, 1, &Localization::CrossesCallback, this);
  fieldboundary_in_image_subscriber_ = nh_.subscribe(config.fieldboundary_in_image_topic, 1,
                                                     &Localization::FieldBoundaryInImageCallback, this);

  pose_particles_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(config.particle_publishing_topic.c_str(),
                                                                             1);
  pose_with_covariance_publisher_ = nh_.advertise<gm::PoseWithCovarianceStamped>("pose_with_covariance", 1);
  lines_publisher_ = nh_.advertise<visualization_msgs::Marker>("lines", 1);
  line_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("line_ratings", 1);
  goal_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("goal_ratings", 1);
  fieldboundary_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("field_boundary_ratings", 1);
  corner_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("corner_ratings", 1);
  t_crossings_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("t_crossings_ratings", 1);
  crosses_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("crosses_ratings", 1);

  ros::Duration(1).sleep();

  service_ = nh_.advertiseService("reset_filter", &Localization::reset_filter_callback, this);

  ROS_INFO_STREAM("Setting path to " << config.map_path_lines);
  lines_.reset(new Map(config.map_path_lines, config));
  goals_.reset(new Map(config.map_path_goals, config));
  field_boundary_.reset(new Map(config.map_path_field_boundary, config));
  corner_.reset(new Map(config.map_path_corners, config));
  t_crossings_map_.reset(new Map(config.map_path_tcrossings, config));
  crosses_map_.reset(new Map(config.map_path_crosses, config));

  line_information_relative_.header.stamp = ros::Time(0);
  goal_relative_.header.stamp = ros::Time(0);
  fieldboundary_relative_.header.stamp = ros::Time(0);
  corners_.header.stamp = ros::Time(0);
  t_crossings_.header.stamp = ros::Time(0);
  crosses_.header.stamp = ros::Time(0);

  robot_pose_observation_model_.reset(
      new RobotPoseObservationModel(
        lines_, goals_, field_boundary_, corner_, t_crossings_map_, crosses_map_, config));
  robot_pose_observation_model_->set_min_weight(config_.min_weight);

  Eigen::Matrix<double, 3, 2> drift_cov;
  drift_cov <<
    // Standard dev of applied drift related to
    // distance, rotation
    config.drift_distance_to_direction, config.drift_roation_to_direction,
    config.drift_distance_to_distance,  config.drift_roation_to_distance,
    config.drift_distance_to_rotation,  config.drift_rotation_to_rotation;

  // Scale drift form drift per second to drift per filter iteration
  drift_cov /= config.publishing_frequency;

  drift_cov.col(0) *= (1 / (config.max_translation / config.publishing_frequency));
  drift_cov.col(1) *= (1 / (config.max_rotation / config.publishing_frequency));

  robot_motion_model_.reset(
      new RobotMotionModel(random_number_generator_, config.diffusion_x_std_dev, config.diffusion_y_std_dev,
                           config.diffusion_t_std_dev,
                           config.diffusion_multiplicator,
                           drift_cov));
  robot_state_distribution_start_left_.reset(new RobotStateDistributionStartLeft(random_number_generator_,
                                                                                 std::make_pair(config.initial_robot_x1,
                                                                                                config
                                                                                                    .initial_robot_y1),
                                                                                 config.initial_robot_t1,
                                                                                 std::make_pair(config.initial_robot_x2,
                                                                                                config
                                                                                                    .initial_robot_y2),
                                                                                 config.initial_robot_t2,
                                                                                 std::make_pair(config.field_x,
                                                                                                config.field_y)));
  robot_state_distribution_start_right_.reset(new RobotStateDistributionStartRight(random_number_generator_,
                                                                                   std::make_pair(config
                                                                                                      .initial_robot_x1,
                                                                                                  config
                                                                                                      .initial_robot_y1),
                                                                                   config.initial_robot_t1,
                                                                                   std::make_pair(config
                                                                                                      .initial_robot_x2,
                                                                                                  config
                                                                                                      .initial_robot_y2),
                                                                                   config.initial_robot_t2,
                                                                                   std::make_pair(config.field_x,
                                                                                                  config.field_y)));
  robot_state_distribution_left_half_.reset(new RobotStateDistributionLeftHalf(random_number_generator_,
                                                                               std::make_pair(config.field_x,
                                                                                              config.field_y)));
  robot_state_distribution_right_half_.reset(new RobotStateDistributionRightHalf(random_number_generator_,
                                                                                 std::make_pair(config.field_x,
                                                                                                config.field_y)));
  robot_state_distribution_position_.reset(new RobotStateDistributionPosition(random_number_generator_,
                                                                              config.initial_robot_x,
                                                                              config.initial_robot_y));
  robot_state_distribution_pose_.reset(new RobotStateDistributionPose(random_number_generator_,
                                                                      config.initial_robot_x,
                                                                      config.initial_robot_y,
                                                                      config.initial_robot_t));

  resampling_.reset(new pf::ImportanceResampling<RobotState>());

  config_ = config;
  if (first_configuration_) {
    first_configuration_ = false;
    ROS_INFO("Trying to initialize particle filter...");
    reset_filter(config_.init_mode);
  }

  publishing_timer_ = nh_.createTimer(static_cast<double>(config.publishing_frequency),
                                      &Localization::run_filter_one_step, this);

}

void Localization::run_filter_one_step(const ros::TimerEvent &e) {
  timer_callback_count_++;
  resampled_ = false;

  // Set the measurements in the observation model
  updateMeasurements();

  // Get the odometry offset since the last cycle
  getMotion();

  if ((config_.filter_only_with_motion and robot_moved) or (!config_.filter_only_with_motion)) {
    robot_pf_->drift(linear_movement_, rotational_movement_);
    robot_pf_->diffuse();
  }

  // Apply ratings corresponding to the observations compared with each partice position
  robot_pf_->measure();

  // Check if its resampling time!
  if (timer_callback_count_ % config_.resampling_interval == 0) {
    robot_pf_->resample();
    timer_callback_count_ = 0;
    resampled_ = true;
  }
  // Publish transforms
  publish_transforms();
  // Publish covariance message
  publish_pose_with_covariance();
  //Publish debug stuff
  if(config_.debug_visualization) {
    publish_debug();
  }

  robot_pose_observation_model_->clear_measurement();
}

void Localization::LineCallback(const hlm::LineInformationRelative &msg) {
  line_information_relative_ = msg;
}

void Localization::GoalCallback(const hlm::GoalRelative &msg) {
  goal_relative_ = msg;
}

void Localization::FieldboundaryCallback(const hlm::FieldBoundaryRelative &msg) {
  fieldboundary_relative_.field_boundary_points.clear();
  fieldboundary_relative_.header = msg.header;
  for (hlm::FieldBoundaryInImage fBinImage : fieldboundary_in_image_) { // find corresponding fb_in_image message
    if (fBinImage.header.stamp == msg.header.stamp) {
      for (int i = 1; i < msg.field_boundary_points.size() - 2; i++) { //ignore most left and right point
        if (fBinImage.field_boundary_points[i].y > 0 && fBinImage.field_boundary_points[i + 1].y
            > 0) { //ignore points that form a line on uppermost row in image
          std::vector<gm::Point> vector = interpolateFieldboundaryPoints(msg.field_boundary_points[i],
                                                                         msg.field_boundary_points[i + 1]);
          for (gm::Point point : vector) {
            fieldboundary_relative_.field_boundary_points.push_back(point);
          }
        }
      }
    }
  }

  for (int i = 1; i < fieldboundary_in_image_.size(); i++) {
    if (fieldboundary_in_image_[i].header.stamp <= msg.header.stamp) {
      fieldboundary_in_image_.erase(fieldboundary_in_image_.begin() + i);
    }
  }
}

void Localization::CornerCallback(const hlm::PixelsRelative &msg) {
  corners_ = msg;
}

void Localization::TCrossingsCallback(const hlm::PixelsRelative &msg) {
  t_crossings_ = msg;
}

void Localization::CrossesCallback(const hlm::PixelsRelative &msg) {
  crosses_ = msg;
}

void Localization::FieldBoundaryInImageCallback(const hlm::FieldBoundaryInImage &msg) {
  fieldboundary_in_image_.push_back(msg);
}

std::vector<gm::Point> Localization::interpolateFieldboundaryPoints(gm::Point point1, gm::Point point2) {

  std::vector<gm::Point> pointsInterpolated;
  double dx = abs(point2.x - point1.x);
  double dy = abs(point2.y - point1.y);

  int steps = 1;

  double stepsizey = dy / steps;
  double stepsizex = dx / steps;

  for (int i = 0; i < steps; i++) {
    gm::Point point;
    point.y = point1.y + (i * stepsizey);
    point.x = point1.x + (i * stepsizex);
    pointsInterpolated.push_back(point);
  }

  return pointsInterpolated;
}

bool Localization::reset_filter_callback(bl::reset_filter::Request &req,
                                         bl::reset_filter::Response &res) {
  if (req.init_mode == 3) {
    reset_filter(req.init_mode, req.x, req.y);
  } else {
    reset_filter(req.init_mode);
  }
  res.success = true;
  return true;
}

void Localization::reset_filter(int distribution) {
  ROS_INFO("reset filter");

  robot_pf_.reset(new particle_filter::ParticleFilter<RobotState>(
      config_.particle_number, robot_pose_observation_model_, robot_motion_model_));

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
      config_.particle_number, robot_pose_observation_model_, robot_motion_model_));

  robot_pf_->setResamplingStrategy(resampling_);

  if (distribution == 3) {
    robot_state_distribution_position_.reset(new RobotStateDistributionPosition(random_number_generator_, x, y));
    robot_pf_->drawAllFromDistribution(robot_state_distribution_position_);
  }

}

void Localization::updateMeasurements() {
  // Sets the measurements in the oservation model
  if (config_.lines_factor && line_information_relative_.header.stamp != last_stamp_lines) {
    robot_pose_observation_model_->set_measurement_lines(line_information_relative_);
  }
  if (config_.goals_factor && goal_relative_.header.stamp != last_stamp_goals) {
    robot_pose_observation_model_->set_measurement_goal(goal_relative_);
  }
  if (config_.field_boundary_factor && fieldboundary_relative_.header.stamp != last_stamp_fb_points) {
    robot_pose_observation_model_->set_measurement_field_boundary(fieldboundary_relative_);
  }
  if (config_.corners_factor && corners_.header.stamp != last_stamp_corners) {
    robot_pose_observation_model_->set_measurement_corners(corners_);
  }
  if (config_.t_crossings_factor && t_crossings_.header.stamp != last_stamp_tcrossings) {
    robot_pose_observation_model_->set_measurement_t_crossings(t_crossings_);
  }
  if (config_.crosses_factor && crosses_.header.stamp != last_stamp_crosses) {
    robot_pose_observation_model_->set_measurement_crosses(crosses_);
  }

  // Set timestamps to mark past messages
  last_stamp_lines = line_information_relative_.header.stamp;
  last_stamp_goals = goal_relative_.header.stamp;
  last_stamp_fb_points = fieldboundary_relative_.header.stamp;
  last_stamp_corners = corners_.header.stamp;
  last_stamp_tcrossings = t_crossings_.header.stamp;
  last_stamp_crosses = crosses_.header.stamp;
}

void Localization::getMotion() {
  robot_moved = false;
  geometry_msgs::TransformStamped transformStampedPast;
  geometry_msgs::TransformStamped transformStampedNow;

  try {

    transformStampedNow = tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0));

    ros::Time past = transformStampedNow.header.stamp - ros::Duration(1.0/(float)config_.publishing_frequency);

    transformStampedPast = tfBuffer.lookupTransform("odom", "base_footprint", past);

    //linear movement
    double global_diff_x, global_diff_y;
    global_diff_x = transformStampedNow.transform.translation.x - transformStampedPast.transform.translation.x;
    global_diff_y = transformStampedNow.transform.translation.y - transformStampedPast.transform.translation.y;

    // Convert to local frame
    auto [polar_rot, polar_dist] = cartesianToPolar(global_diff_x, global_diff_y);
    auto [local_movement_x, local_movement_y] = polarToCartesian(
      polar_rot - tf::getYaw(transformStampedPast.transform.rotation), polar_dist);
    linear_movement_.x = local_movement_x;
    linear_movement_.y = local_movement_y;
    linear_movement_.z = 0;

    //rotational movement
    rotational_movement_.x = 0;
    rotational_movement_.y = 0;
    rotational_movement_.z = tf::getYaw(transformStampedNow.transform.rotation) -
        tf::getYaw(transformStampedPast.transform.rotation);

    //check if robot moved
    if (linear_movement_.x > config_.min_motion_linear or linear_movement_.y > config_.min_motion_linear or
        rotational_movement_.z > config_.min_motion_angular) {
      robot_moved = true;
    }

  }
  catch (const tf2::TransformException &ex) {
    ROS_WARN("Could not aquire motion for odom transforms: %s", ex.what());
  }
}

void Localization::publish_transforms() {

  //get estimate and covariance
  estimate_ = robot_pf_->getBestXPercentEstimate(config_.percentage_best_particles);
  std::vector<double> estimate_cov_ = robot_pf_->getCovariance(config_.percentage_best_particles);

  //calculate quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, estimate_.getTheta());
  q.normalize();

  //////////////////////
  //publish transforms//
  //////////////////////

  //publish localization tf, not the odom offset
  geometry_msgs::TransformStamped localization_transform;
  localization_transform.header.stamp = ros::Time::now();
  localization_transform.header.frame_id = "/map";
  localization_transform.child_frame_id = config_.publishing_frame;
  localization_transform.transform.translation.x = estimate_.getXPos();
  localization_transform.transform.translation.y = estimate_.getYPos();
  localization_transform.transform.translation.z = 0.0;
  localization_transform.transform.rotation.x = q.x();
  localization_transform.transform.rotation.y = q.y();
  localization_transform.transform.rotation.z = q.z();
  localization_transform.transform.rotation.w = q.w();
  br.sendTransform(localization_transform);

  try{
    //publish odom localisation offset
    geometry_msgs::TransformStamped odom_transform = tfBuffer.lookupTransform("odom", "base_footprint", ros::Time(0));
    geometry_msgs::TransformStamped map_odom_transform;

    map_odom_transform.header.stamp = odom_transform.header.stamp;
    map_odom_transform.header.frame_id = "/map";
    map_odom_transform.child_frame_id = "/odom";

    //calculate odom offset
    tf2::Transform odom_transform_tf, localization_transform_tf, map_tf;
    tf2::fromMsg(odom_transform.transform, odom_transform_tf);
    tf2::fromMsg(localization_transform.transform, localization_transform_tf);
    map_tf = localization_transform_tf * odom_transform_tf.inverse();

    map_odom_transform.transform = tf2::toMsg(map_tf);

    br.sendTransform(map_odom_transform);
  }
  catch (const tf2::TransformException &ex) {
    ROS_WARN("Odom not available, therefore odom offset can not be published: %s", ex.what());
  }
}

void Localization::publish_pose_with_covariance() {

  std::vector<double> cov_mat = robot_pf_->getCovariance(config_.percentage_best_particles);

  gm::PoseWithCovarianceStamped estimateMsg;

  estimateMsg.pose.pose.orientation.w = 1;
  estimateMsg.pose.pose.orientation.x = 0;
  estimateMsg.pose.pose.orientation.y = 0;
  estimateMsg.pose.pose.orientation.z = 0;
  estimateMsg.pose.pose.position.x = 0;
  estimateMsg.pose.pose.position.y = 0;

  std::vector<double> covariance;

  for (int i = 0; i < 36; i++) {
    estimateMsg.pose.covariance[i] = cov_mat[i];
  }

  estimateMsg.header.frame_id = config_.publishing_frame;

  pose_with_covariance_publisher_.publish(estimateMsg);
}

void Localization::publish_debug() {
  // Show a marker for each partikle
  publish_particle_markers();
  // Show ratings for each used class
  publish_ratings();
}

void Localization::publish_particle_markers() {
  //publish particle markers
  std_msgs::ColorRGBA red;
  red.r = 1;
  red.g = 0;
  red.b = 0;
  red.a = 1;
  pose_particles_publisher_.publish(robot_pf_->renderMarkerArray("pose_marker", "/map",
                                                                  ros::Duration(1),
                                                                  red));
}

void Localization::publish_ratings() {
  if (config_.lines_factor) {
    // Publish line ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_lines(), 0.1, "line_ratings", lines_, line_ratings_publisher_);
  }
  if (config_.goals_factor) {
    // Publish goal ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_goals(), 0.2, "goal_ratings", goals_, goal_ratings_publisher_);
  }
  if (config_.field_boundary_factor) {
    // Publish field boundary ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_field_boundary(), 0.2, "field_boundary_ratings", field_boundary_, fieldboundary_ratings_publisher_);
  }
  if (config_.corners_factor) {
    // Publish corner ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_corners(), 0.1, "corner_ratings", corner_, corner_ratings_publisher_);
  }
  if (config_.t_crossings_factor) {
    // Publish t-crossing ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_t_crossings(), 0.2, "t_crossing_ratings", t_crossings_map_, t_crossings_ratings_publisher_);
  }
  if (config_.crosses_factor) {
    // Publish cross ratings
    publish_debug_rating(robot_pose_observation_model_->get_measurement_crosses(), 0.2, "cross_ratings", crosses_map_, crosses_ratings_publisher_);
  }
}

void Localization::publish_debug_rating(
    std::vector<std::pair<double, double>> measurements, 
    double scale, 
    const char name[], 
    std::shared_ptr<Map> map, 
    ros::Publisher &publisher) {

  RobotState best_estimate = robot_pf_->getBestXPercentEstimate(config_.percentage_best_particles);
  double rating = 0;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = name;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.scale.x = scale;
  marker.scale.y = scale;

  for (std::pair<double, double> &measurement : measurements) {
    //lines are in polar form!
    std::pair<double, double> observationRelative;

    observationRelative = map->observationRelative(
      measurement, best_estimate.getXPos(), best_estimate.getYPos(), best_estimate.getTheta());
    double occupancy = map->get_occupancy(observationRelative.first, observationRelative.second);

    geometry_msgs::Point point;
    point.x = observationRelative.first;
    point.y = observationRelative.second;
    point.z = 0;

    std_msgs::ColorRGBA color;
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
  publisher.publish(marker);
}
