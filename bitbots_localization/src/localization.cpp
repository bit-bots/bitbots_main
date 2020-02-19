//
// Created by judith on 08.03.19.
//

#include "bitbots_localization/localization.h"

Localization::Localization() : line_points_(), tfListener(tfBuffer) {
  ROS_DEBUG("localization");

}

void Localization::dynamic_reconfigure_callback(hll::LocalizationConfig &config, uint32_t config_level) {
  pose_publisher_ = nh_.advertise<hll::Evaluation>("pose", 1);

  line_subscriber_ = nh_.subscribe(config.line_topic, 1, &Localization::LineCallback, this);
  non_line_subscriber_ = nh_.subscribe(config.non_line_topic, 1, &Localization::NonLineCallback, this);
  goal_subscriber_ = nh_.subscribe(config.goal_topic, 1, &Localization::GoalCallback, this);
  fieldboundary_subscriber_ = nh_.subscribe(config.fieldboundary_topic, 1, &Localization::FieldboundaryCallback,
                                            this);
  corners_subscriber_ = nh_.subscribe(config.corners_topic, 1, &Localization::CornerCallback, this);
  t_crossings_subscriber_ = nh_.subscribe(config.tcrossings_topic, 1, &Localization::TCrossingsCallback, this);
  crosses_subscriber_ = nh_.subscribe(config.crosses_topic, 1, &Localization::CrossesCallback, this);
  cam_info_subscriber_ = nh_.subscribe(config.cam_info_topic, 1, &Localization::CamInfoCallback, this);
  fieldboundary_in_image_subscriber_ = nh_.subscribe(config.fieldboundary_in_image_topic, 1,
                                                     &Localization::FieldBoundaryInImageCallback, this);

  pose_particles_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(config.particle_publishing_topic.c_str(),
                                                                             1);
  pose_with_covariance_publisher_ = nh_.advertise<gm::PoseWithCovarianceStamped>("pose_with_covariance", 1);
  lines_publisher_ = nh_.advertise<visualization_msgs::Marker>("lines", 1);
  line_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("line_ratings", 1);
  goal_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("goal_ratings", 1);
  fieldboundary_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("field_boundary_ratings", 1);
  non_line_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("non_line_ratings", 1);
  corner_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("corner_ratings", 1);
  t_crossings_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("t_crossings_ratings", 1);
  crosses_ratings_publisher_ = nh_.advertise<visualization_msgs::Marker>("crosses_ratings", 1);

  ros::Duration(1).sleep();

  service_ = nh_.advertiseService("reset_filter", &Localization::reset_filter_callback, this);

  lines_.reset(new Map(config.map_path_lines));
  ROS_INFO_STREAM("Setting path to " << config.map_path_lines);
  goals_.reset(new Map(config.map_path_goals));
  field_boundary_.reset(new Map(config.map_path_field_boundary));
  corner_.reset(new Map(config.map_path_corners));
  t_crossings_map_.reset(new Map(config.map_path_tcrossings));
  crosses_map_.reset(new Map(config.map_path_crosses));

  line_information_relative_.header.stamp = ros::Time(0);
  goal_relative_.header.stamp = ros::Time(0);
  fieldboundary_relative_.header.stamp = ros::Time(0);
  corners_.header.stamp = ros::Time(0);
  t_crossings_.header.stamp = ros::Time(0);
  crosses_.header.stamp = ros::Time(0);

  robot_pose_observation_model_.reset(
      new RobotPoseObservationModel(lines_, goals_, field_boundary_, corner_, t_crossings_map_, crosses_map_));
  robot_pose_observation_model_->set_min_weight(config_.min_weight);

  robot_motion_model_.reset(
      new RobotMotionModel(random_number_generator_, config.diffusion_x_std_dev, config.diffusion_y_std_dev,
                           config.diffusion_t_std_dev,
                           config.diffusion_multiplicator));
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
  if (!valid_configuration_) {
    valid_configuration_ = true;
    ROS_INFO("Trying to initialize particle filter...");
    init();
  }

  publishing_timer_ = nh_.createTimer(static_cast<double>(config.publishing_frequency),
                                      &Localization::publishing_timer_callback, this);

}

void Localization::publishing_timer_callback(const ros::TimerEvent &e) {
  timer_callback_count++;
  resampled = 0;

  if (config_.use_lines && line_information_relative_.header.stamp != last_stamp_lines) {
    robot_pose_observation_model_->set_measurement_lines(line_information_relative_);
  }
  if (config_.use_goals && goal_relative_.header.stamp != last_stamp_goals) {
    robot_pose_observation_model_->set_measurement_goal(goal_relative_);
  }
  if (config_.use_fieldboundary && fieldboundary_relative_.header.stamp != last_stamp_fb_points) {
    robot_pose_observation_model_->set_measurement_field_boundary(fieldboundary_relative_);
  }
  if (config_.use_corners && corners_.header.stamp != last_stamp_corners) {
    robot_pose_observation_model_->set_measurement_corners(corners_);
  }
  if (config_.use_tcrossings && t_crossings_.header.stamp != last_stamp_tcrossings) {
    robot_pose_observation_model_->set_measurement_t_crossings(t_crossings_);
  }
  if (config_.use_crosses && crosses_.header.stamp != last_stamp_crosses) {
    robot_pose_observation_model_->set_measurement_crosses(crosses_);
  }
  last_stamp_lines = line_information_relative_.header.stamp;
  last_stamp_goals = goal_relative_.header.stamp;
  last_stamp_fb_points = fieldboundary_relative_.header.stamp;
  last_stamp_corners = corners_.header.stamp;
  last_stamp_tcrossings = t_crossings_.header.stamp;
  last_stamp_crosses = crosses_.header.stamp;

  getMotion();

  if ((config_.filter_only_with_motion and robot_moved) or (!config_.filter_only_with_motion)) {

    robot_pf_->drift(movement_, movement2_);
    robot_pf_->diffuse();
  }

  robot_pf_->measure();

  if (timer_callback_count % config_.resampling_interval == 0) {
    robot_pf_->resample();
    timer_callback_count = 0;
    resampled = 1;
  }
  publish_pose();
  publish_pose_with_covariance();

  if (config_.use_lines) {
    publish_line_ratings();
  }
  if (config_.use_goals) {
    publish_goal_ratings();
  }
  if (config_.use_fieldboundary) {
    publish_field_boundary_ratings();
  }
  if (config_.use_corners) {
    publish_corner_ratings();
  }
  if (config_.use_tcrossings) {
    publish_t_crossings_ratings();
  }
  if (config_.use_crosses) {
    publish_crosses_ratings();
  }

  robot_pose_observation_model_->clear_measurement();

}

void Localization::LineCallback(const hlm::LineInformationRelative &msg) {
  line_information_relative_ = msg;
}

void Localization::NonLineCallback(const hlm::LineInformationRelative &msg) {
  non_line_information_relative_ = msg;

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

void Localization::CamInfoCallback(const sensor_msgs::CameraInfo &msg) {
  cam_info_ = msg;

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

bool Localization::reset_filter_callback(hll::reset_filter::Request &req,
                                         hll::reset_filter::Response &res) {
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

void Localization::init() {
  if (!valid_configuration_) {
    ROS_ERROR(
        "You tried to initialize the particle filter with an invalid configuration!\n "
        "The dynamic_reconfigure_callback has to be called at least once with a valid configuration before initializing the particle filter!");
    return;
  }
  ROS_INFO("init");

  reset_filter(config_.init_mode); // right half
}

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

void Localization::getMotion() {
  robot_moved = false;
  //get transforms
  ros::Time now = ros::Time(0);
  ros::Time past = ros::Time::now() -
      ros::Duration(0.04);

  geometry_msgs::TransformStamped transformStampedPast;
  geometry_msgs::TransformStamped transformStampedNow;

  movement2_.x = 0;
  movement2_.y = 0;
  movement2_.z = 0;

  try {

    transformStampedPast = tfBuffer.lookupTransform("odom", "base_footprint", past);
    transformStampedNow = tfBuffer.lookupTransform("odom", "base_footprint", now);

    geometry_msgs::Vector3 past_;
    geometry_msgs::Vector3 now_;

    past_.x = transformStampedPast.transform.translation.x;
    past_.y = transformStampedPast.transform.translation.y;
    past_.z = tf::getYaw(transformStampedPast.transform.rotation);

    now_.x = transformStampedNow.transform.translation.x;
    now_.y = transformStampedNow.transform.translation.y;
    now_.z = tf::getYaw(transformStampedNow.transform.rotation);

    movement_.x = atan2(now_.y - past_.y, now_.x - past_.x) - past_.z; //deltaRot1
    movement_.y = hypot(past_.x - now_.x, past_.y - now_.y); //deltaTrans
    movement_.z = now_.z - past_.z - movement_.x; // deltaRot2

    //rotational movement
    geometry_msgs::Vector3 rotation_;
    geometry_msgs::Vector3 translation_;
    rotation_.x = 0;
    rotation_.y = 0;
    rotation_.z = tf::getYaw(transformStampedNow.transform.rotation) -
        tf::getYaw(transformStampedPast.transform.rotation);

    //linear movement
    translation_.x = transformStampedNow.transform.translation.x - transformStampedPast.transform.translation.x;
    translation_.y = transformStampedNow.transform.translation.y - transformStampedPast.transform.translation.y;
    translation_.z =
        transformStampedNow.transform.translation.z - transformStampedPast.transform.translation.z; // oder 0?

    if (translation_.x > config_.min_motion_linear or translation_.y > config_.min_motion_linear or
        rotation_.z > config_.min_motion_angular) {
      robot_moved = true;
      // ROS_INFO_STREAM("robot moved");
    }

  }
  catch (const tf2::TransformException &ex) {
    ROS_WARN("get_motion: %s", ex.what());
  }

}

void Localization::publish_pose() { //  and particles and map frame
  if (config_.debug_visualization) {
    //publish particles
    std_msgs::ColorRGBA red;
    red.r = 1;
    red.g = 0;
    red.b = 0;
    red.a = 1;
    pose_particles_publisher_.publish(robot_pf_->renderMarkerArray("pose_marker", "/map",
                                                                   ros::Duration(1),
                                                                   red));
    //get estimates and covariances
    best_estimate_ = robot_pf_->getBestXPercentEstimate(0.2); // best particle
    best_estimate_5_ = robot_pf_->getBestXPercentEstimate(5);
    std::vector<double> cov_estimate_5 = robot_pf_->getCovariance(5);
    best_estimate_10_ = robot_pf_->getBestXPercentEstimate(10);
    std::vector<double> cov_estimate_10 = robot_pf_->getCovariance(10);
    best_estimate_20_ = robot_pf_->getBestXPercentEstimate(20);
    std::vector<double> cov_estimate_20 = robot_pf_->getCovariance(20);
    best_estimate_mean_ = robot_pf_->getBestXPercentEstimate(100);
    std::vector<double> cov_mean = robot_pf_->getCovariance(100);

    //calculate quaternions
    tf2::Quaternion q;
    q.setRPY(0, 0, best_estimate_.getTheta());
    q.normalize();

    tf2::Quaternion q5;
    q5.setRPY(0, 0, best_estimate_5_.getTheta());
    q5.normalize();

    tf2::Quaternion q10;
    q10.setRPY(0, 0, best_estimate_10_.getTheta());
    q10.normalize();

    tf2::Quaternion q20;
    q20.setRPY(0, 0, best_estimate_20_.getTheta());
    q20.normalize();

    tf2::Quaternion qmean;
    qmean.setRPY(0, 0, best_estimate_mean_.getTheta());
    qmean.normalize();

    //publish transforms
    geometry_msgs::TransformStamped trans_best_estimate;
    trans_best_estimate.header.stamp = ros::Time::now();
    trans_best_estimate.header.frame_id = "/map";
    trans_best_estimate.child_frame_id = "/best_estimate";
    trans_best_estimate.transform.translation.x = best_estimate_.getXPos();
    trans_best_estimate.transform.translation.y = best_estimate_.getYPos();
    trans_best_estimate.transform.translation.z = 0.0;
    trans_best_estimate.transform.rotation.x = q.x();
    trans_best_estimate.transform.rotation.y = q.y();
    trans_best_estimate.transform.rotation.z = q.z();
    trans_best_estimate.transform.rotation.w = q.w();
    br.sendTransform(trans_best_estimate);

    geometry_msgs::TransformStamped trans_best_estimate_5;
    trans_best_estimate_5.header.stamp = ros::Time::now();
    trans_best_estimate_5.header.frame_id = "/map";
    trans_best_estimate_5.child_frame_id = "/best_estimate_5";
    trans_best_estimate_5.transform.translation.x = best_estimate_5_.getXPos();
    trans_best_estimate_5.transform.translation.y = best_estimate_5_.getYPos();
    trans_best_estimate_5.transform.translation.z = 0.0;
    trans_best_estimate_5.transform.rotation.x = q5.x();
    trans_best_estimate_5.transform.rotation.y = q5.y();
    trans_best_estimate_5.transform.rotation.z = q5.z();
    trans_best_estimate_5.transform.rotation.w = q5.w();
    br.sendTransform(trans_best_estimate_5);

    geometry_msgs::TransformStamped trans_best_estimate_10;
    trans_best_estimate_10.header.stamp = ros::Time::now();
    trans_best_estimate_10.header.frame_id = "/map";
    trans_best_estimate_10.child_frame_id = config_.publishing_frame; // localization_estimate
    trans_best_estimate_10.transform.translation.x = best_estimate_10_.getXPos();
    trans_best_estimate_10.transform.translation.y = best_estimate_10_.getYPos();
    trans_best_estimate_10.transform.translation.z = 0.0;
    trans_best_estimate_10.transform.rotation.x = q10.x();
    trans_best_estimate_10.transform.rotation.y = q10.y();
    trans_best_estimate_10.transform.rotation.z = q10.z();
    trans_best_estimate_10.transform.rotation.w = q10.w();
    br.sendTransform(trans_best_estimate_10);

    geometry_msgs::TransformStamped trans_best_estimate_20;
    trans_best_estimate_20.header.stamp = ros::Time::now();
    trans_best_estimate_20.header.frame_id = "/map";
    trans_best_estimate_20.child_frame_id = "/best_estimate_20";
    trans_best_estimate_20.transform.translation.x = best_estimate_20_.getXPos();
    trans_best_estimate_20.transform.translation.y = best_estimate_20_.getYPos();
    trans_best_estimate_20.transform.translation.z = 0.0;
    trans_best_estimate_20.transform.rotation.x = q20.x();
    trans_best_estimate_20.transform.rotation.y = q20.y();
    trans_best_estimate_20.transform.rotation.z = q20.z();
    trans_best_estimate_20.transform.rotation.w = q20.w();
    br.sendTransform(trans_best_estimate_20);

    geometry_msgs::TransformStamped trans_mean;
    trans_mean.header.stamp = ros::Time::now();
    trans_mean.header.frame_id = "/map";
    trans_mean.child_frame_id = "/mean";
    trans_mean.transform.translation.x = best_estimate_mean_.getXPos();
    trans_mean.transform.translation.y = best_estimate_mean_.getYPos();
    trans_mean.transform.translation.z = 0.0;
    trans_mean.transform.rotation.x = qmean.x();
    trans_mean.transform.rotation.y = qmean.y();
    trans_mean.transform.rotation.z = qmean.z();
    trans_mean.transform.rotation.w = qmean.w();
    br.sendTransform(trans_mean);

    //fill and publish evaluation message
    bitbots_localization::Evaluation estimateMsg;
    estimateMsg.header.frame_id = config_.publishing_frame;
    estimateMsg.header.stamp = trans_best_estimate.header.stamp;

    for (int i = 0; i < 36; i++) {
      estimateMsg.cov_estimate_5[i] = cov_estimate_5[i];
      estimateMsg.cov_estimate_10[i] = cov_estimate_10[i];
      estimateMsg.cov_estimate_20[i] = cov_estimate_20[i];
      estimateMsg.cov_mean[i] = cov_mean[i];
    }

    estimateMsg.lines = RobotPoseObservationModel::number_lines;
    estimateMsg.goals = RobotPoseObservationModel::number_goals;
    estimateMsg.fb_points = RobotPoseObservationModel::number_fb_points;
    estimateMsg.corners = RobotPoseObservationModel::number_corners;
    estimateMsg.tcrossings = RobotPoseObservationModel::number_tcrossings;
    estimateMsg.crosses = RobotPoseObservationModel::number_corners;

    estimateMsg.resampled = resampled;

    pose_publisher_.publish(estimateMsg);
  }
}

void Localization::publish_pose_with_covariance() {

  if (config_.debug_visualization) {

    std::vector<double> cov_mat = robot_pf_->getCovariance(100);

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

    estimateMsg.header.frame_id = "mean";

    pose_with_covariance_publisher_.publish(estimateMsg);
  }
}

// todo refactor all publish ratings functions and move to debug class

void Localization::publish_line_ratings() {
  if (config_.debug_visualization) {
    std::vector<std::pair<double, double>> lines = robot_pose_observation_model_->get_measurement_lines();
    best_estimate_ = robot_pf_->getBestState();
    double rating = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "line_ratings";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;

    for (std::pair<double, double> &line : lines) {
      //lines are in polar form!
      std::pair<double, double> lineRelative;

      lineRelative = lines_->observationRelative(line, best_estimate_.getXPos(), best_estimate_.getYPos(),
                                                 best_estimate_.getTheta());
      double occupancy = lines_->get_occupancy(lineRelative.first, lineRelative.second);

      geometry_msgs::Point point;
      point.x = lineRelative.first;
      point.y = lineRelative.second;
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
    line_ratings_publisher_.publish(marker);
  }
}

void Localization::publish_goal_ratings() {
  if (config_.debug_visualization) {
    std::vector<std::pair<double, double>> goals = robot_pose_observation_model_->get_measurement_goals();
    best_estimate_ = robot_pf_->getBestXPercentEstimate(config_.percentage_best_particles);
    double rating = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "goal_ratings";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.lifetime = ros::Duration(1.0);

    for (std::pair<double, double> &goal : goals) {
      //lines are in polar form!
      std::pair<double, double> goalRelative;

      goalRelative = goals_->observationRelative(goal, best_estimate_.getXPos(), best_estimate_.getYPos(),
                                                 best_estimate_.getTheta());
      double occupancy = goals_->get_occupancy(goalRelative.first, goalRelative.second);

      geometry_msgs::Point point;
      point.x = goalRelative.first;
      point.y = goalRelative.second;
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
    goal_ratings_publisher_.publish(marker);
  }
}

void Localization::publish_field_boundary_ratings() {
  if (config_.debug_visualization) {
    std::vector<std::pair<double, double>> fb_points = robot_pose_observation_model_->get_measurement_field_boundary();
    best_estimate_ = robot_pf_->getBestXPercentEstimate(config_.percentage_best_particles);
    double rating = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "fb_ratings";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.lifetime = ros::Duration(1.0);

    for (std::pair<double, double> &fb : fb_points) {
      //lines are in polar form!
      std::pair<double, double> fbRelative;

      fbRelative = field_boundary_->observationRelative(fb, best_estimate_.getXPos(), best_estimate_.getYPos(),
                                                        best_estimate_.getTheta());
      double occupancy = field_boundary_->get_occupancy(fbRelative.first, fbRelative.second);
      geometry_msgs::Point point;
      point.x = fbRelative.first;
      point.y = fbRelative.second;
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
    fieldboundary_ratings_publisher_.publish(marker);
  }
}

void Localization::publish_corner_ratings() {
  if (config_.debug_visualization) {
    std::vector<std::pair<double, double>> corners = robot_pose_observation_model_->get_measurement_corners();
    best_estimate_ = robot_pf_->getBestXPercentEstimate(config_.percentage_best_particles);
    double rating = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "corner_ratings";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.lifetime = ros::Duration(1.0);

    for (std::pair<double, double> &corner : corners) {
      //lines are in polar form!
      std::pair<double, double> cornerRelative;

      cornerRelative = corner_->observationRelative(corner, best_estimate_.getXPos(), best_estimate_.getYPos(),
                                                    best_estimate_.getTheta());
      double occupancy = corner_->get_occupancy(cornerRelative.first, cornerRelative.second);
      geometry_msgs::Point point;
      point.x = cornerRelative.first;
      point.y = cornerRelative.second;
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
    corner_ratings_publisher_.publish(marker);
  }
}

void Localization::publish_t_crossings_ratings() {
  if (config_.debug_visualization) {
    std::vector<std::pair<double, double>> tcrossings = robot_pose_observation_model_->get_measurement_t_crossings();
    best_estimate_ = robot_pf_->getBestXPercentEstimate(config_.percentage_best_particles);
    double rating = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "tcrossings_ratings";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.lifetime = ros::Duration(1.0);

    for (std::pair<double, double> &tcrossing : tcrossings) {
      //lines are in polar form!
      std::pair<double, double> tcrossingRelative;

      tcrossingRelative = t_crossings_map_->observationRelative(tcrossing, best_estimate_.getXPos(),
                                                                best_estimate_.getYPos(),
                                                                best_estimate_.getTheta());
      double occupancy = t_crossings_map_->get_occupancy(tcrossingRelative.first, tcrossingRelative.second);
      geometry_msgs::Point point;
      point.x = tcrossingRelative.first;
      point.y = tcrossingRelative.second;
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
    t_crossings_ratings_publisher_.publish(marker);
  }
}

void Localization::publish_crosses_ratings() {
  if (config_.debug_visualization) {
    std::vector<std::pair<double, double>> corners = robot_pose_observation_model_->get_measurement_crosses();
    best_estimate_ = robot_pf_->getBestXPercentEstimate(config_.percentage_best_particles);
    double rating = 0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "crosses_ratings";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.lifetime = ros::Duration(1.0);

    for (std::pair<double, double> &corner : corners) {
      //lines are in polar form!
      std::pair<double, double> cornerRelative;

      cornerRelative = crosses_map_->observationRelative(corner, best_estimate_.getXPos(),
                                                         best_estimate_.getYPos(),
                                                         best_estimate_.getTheta());
      double occupancy = crosses_map_->get_occupancy(cornerRelative.first, cornerRelative.second);

      geometry_msgs::Point point;
      point.x = cornerRelative.first;
      point.y = cornerRelative.second;
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
    crosses_ratings_publisher_.publish(marker);
  }
}

