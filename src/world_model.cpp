#include "bitbots_world_model/world_model.h"


WorldModel::WorldModel() :
        nh_(),
        valid_configuration_(false),
        transform_listener_(transform_buffer_) {
    ROS_INFO("Created Bit-Bots world model");
}

void WorldModel::dynamic_reconfigure_callback(
        bitbots_world_model::WorldModelConfig& config, uint32_t level) {
    ROS_INFO("Dynamic reconfigure callback was called...");

    // updating topic names when neccessary
    if (config.obstacles_topic != config_.obstacles_topic) {
        obstacle_subscriber_ = nh_.subscribe(config.obstacles_topic.c_str(), 1,
                &WorldModel::obstacles_callback, this);
    }
    if (config.ball_topic != config_.ball_topic) {
        ball_subscriber_ = nh_.subscribe(
                config.ball_topic.c_str(), 1, &WorldModel::ball_callback, this);
    }
    if (config.team_data_topic != config_.team_data_topic) {
        team_data_subscriber_ = nh_.subscribe(config.team_data_topic.c_str(), 1,
                &WorldModel::team_data_callback, this);
    }
    if (config.local_model_topic != config_.local_model_topic) {
        local_model_publisher_ =
                nh_.advertise<hlm::Model>(config.local_model_topic.c_str(), 1);
    }
    if (config.global_model_topic != config_.global_model_topic) {
        global_model_publisher_ =
                nh_.advertise<hlm::Model>(config.global_model_topic.c_str(), 1);
    }
    if (config.local_particles_topic != config_.local_particles_topic) {
        local_particles_publisher_ = nh_.advertise<visualization_msgs::Marker>(
                config.local_particles_topic.c_str(), 1);
    }
    if (config.reset_filters_service_name !=
            config_.reset_filters_service_name) {
        reset_filters_service_ =
                nh_.advertiseService(config.reset_filters_service_name,
                        &WorldModel::reset_filters_callback, this);
    }

    // initializing timer
    publishing_timer_ = nh_.createTimer(
            ros::Duration(
                    1.0 / static_cast<double>(config.publishing_frequency)),
            &WorldModel::publishing_timer_callback, this);

    // team color
    if (config.team_color != config_.team_color) {
        if (config.team_color == hlm::ObstacleRelative::ROBOT_MAGENTA) {
            team_color_ = config.team_color;
            opponent_color_ = hlm::ObstacleRelative::ROBOT_CYAN;
            ROS_INFO("Switched team color to red");
        } else if (config.team_color == hlm::ObstacleRelative::ROBOT_CYAN) {
            team_color_ = config.team_color;
            opponent_color_ = hlm::ObstacleRelative::ROBOT_MAGENTA;
            ROS_INFO("Switched team color to blue");
        } else {
            ROS_INFO_STREAM(
                    "Could not set the input \""
                    << config.team_color
                    << "\" as team color. the value has to correspond with "
                    << "either ROBOT_MAGENTA or ROBOT_CYAN set in the "
                    << "ObstacleRelative message");
        }
    }

    // initializing observation models
    // local
    local_ball_observation_model_.reset(new LocalFcnnObservationModel());
    local_ball_observation_model_->set_min_weight(config.local_ball_min_weight);
    local_ball_observation_model_->set_k(1);
    local_mate_observation_model_.reset(new LocalRobotObservationModel());
    local_mate_observation_model_->set_min_weight(config.local_mate_min_weight);
    local_opponent_observation_model_.reset(new LocalRobotObservationModel());
    local_opponent_observation_model_->set_min_weight(
            config.local_opponent_min_weight);
    local_obstacle_observation_model_.reset(
            new LocalObstacleObservationModel());
    local_obstacle_observation_model_->set_min_weight(
            config.local_obstacle_min_weight);
    // global
    global_ball_observation_model_.reset(new GlobalBallObservationModel());
    global_ball_observation_model_->set_min_weight(config.global_ball_min_weight);
    global_mate_observation_model_.reset(new GlobalRobotObservationModel());
    global_mate_observation_model_->set_min_weight(config.global_mate_min_weight);
    global_opponent_observation_model_.reset(new GlobalRobotObservationModel());
    global_opponent_observation_model_->set_min_weight(
            config.global_opponent_min_weight);

    // initializing movement models
    // local 
    local_ball_movement_model_.reset(new LocalRobotMovementModel(
            random_number_generator_, config.local_ball_diffusion_x_std_dev,
            config.local_ball_diffusion_y_std_dev,
            config.local_ball_diffusion_multiplicator));
    local_mate_movement_model_.reset(new LocalRobotMovementModel(
            random_number_generator_, config.local_mate_diffusion_x_std_dev,
            config.local_mate_diffusion_y_std_dev,
            config.local_mate_diffusion_multiplicator));
    local_opponent_movement_model_.reset(new LocalRobotMovementModel(
            random_number_generator_, config.local_opponent_diffusion_x_std_dev,
            config.local_opponent_diffusion_y_std_dev,
            config.local_opponent_diffusion_multiplicator));
    local_obstacle_movement_model_.reset(new LocalObstacleMovementModel(
            random_number_generator_, config.local_obstacle_diffusion_x_std_dev,
            config.local_obstacle_diffusion_y_std_dev,
            config.local_obstacle_diffusion_multiplicator));
    // global 
    global_ball_movement_model_.reset(new GlobalBallMovementModel(
            random_number_generator_, config.global_ball_diffusion_x_std_dev,
            config.global_ball_diffusion_y_std_dev,
            config.global_ball_diffusion_multiplicator));
    global_mate_movement_model_.reset(new GlobalRobotMovementModel(
            random_number_generator_, config.global_mate_diffusion_x_std_dev,
            config.global_mate_diffusion_y_std_dev,
            config.global_mate_diffusion_multiplicator));
    global_opponent_movement_model_.reset(new GlobalRobotMovementModel(
            random_number_generator_, config.global_opponent_diffusion_x_std_dev,
            config.global_opponent_diffusion_y_std_dev,
            config.global_opponent_diffusion_multiplicator));

    // initializing state distributions
    // local
    local_ball_state_distribution_.reset(new LocalPositionStateDistribution(
            random_number_generator_,
            std::make_pair(config.initial_robot_x, config.initial_robot_y),
            std::make_pair(config.field_height, config.field_width)));
    local_mate_state_distribution_.reset(new LocalPositionStateDistribution(
            random_number_generator_,
            std::make_pair(config.initial_robot_x, config.initial_robot_y),
            std::make_pair(config.field_height, config.field_width)));
    local_opponent_state_distribution_.reset(new LocalPositionStateDistribution(
            random_number_generator_,
            std::make_pair(config.initial_robot_x, config.initial_robot_y),
            std::make_pair(config.field_height, config.field_width)));
    local_obstacle_state_distribution_.reset(
            new LocalPositionStateWDistribution(random_number_generator_,
                    std::make_pair(
                            config.initial_robot_x, config.initial_robot_y),
                    std::make_pair(config.field_height, config.field_width),
                    config.local_obstacle_min_width,
                    config.local_obstacle_max_width));
    // global
    global_ball_state_distribution_.reset(new GlobalPositionStateDistribution(
            random_number_generator_,
            std::make_pair(config.initial_robot_x, config.initial_robot_y),
            std::make_pair(config.field_height, config.field_width)));
    global_mate_state_distribution_.reset(new GlobalPositionStateDistribution(
            random_number_generator_,
            std::make_pair(config.initial_robot_x, config.initial_robot_y),
            std::make_pair(config.field_height, config.field_width)));
    global_opponent_state_distribution_.reset(new GlobalPositionStateDistribution(
            random_number_generator_,
            std::make_pair(config.initial_robot_x, config.initial_robot_y),
            std::make_pair(config.field_height, config.field_width)));

    // setting particle counts
    if (config.local_obstacle_particle_number !=
                    config_.local_obstacle_particle_number ||
            config.local_mate_particle_number !=
                    config_.local_mate_particle_number ||
            config.local_opponent_particle_number !=
                    config_.local_opponent_particle_number) {
        ROS_INFO_STREAM(
                "You changed the particle number to the following: "
                << "\nlocal_ball_particle_number: "
                << config.local_ball_particle_number
                << "\nlocal_obstacle_particle_number: "
                << config.local_obstacle_particle_number
                << "\nlocal_mate_particle_number: "
                << config.local_mate_particle_number
                << "\nlocal_opponent_particle_number: "
                << config.local_opponent_particle_number
                << "\nTo use the updated particle numbers, you need to reset the filters.");
    }

    // initializing resampling methods
    local_ball_resampling_.reset(new ImportanceResamplingWE<PositionState>(
            static_cast<int>(config.local_ball_particle_number *
                             config.local_ball_explorer_rate),
            local_ball_state_distribution_));
    local_mate_resampling_.reset(new ImportanceResamplingWE<PositionState>(
            static_cast<int>(config.local_mate_particle_number *
                             config.local_mate_explorer_rate),
            local_mate_state_distribution_));
    local_opponent_resampling_.reset(new ImportanceResamplingWE<PositionState>(
            static_cast<int>(config.local_opponent_particle_number *
                             config.local_opponent_explorer_rate),
            local_opponent_state_distribution_));
    local_obstacle_resampling_.reset(new ImportanceResamplingWE<PositionStateW>(
            static_cast<int>(config.local_obstacle_particle_number *
                             config.local_obstacle_explorer_rate),
            local_obstacle_state_distribution_));

    config_ = config;
    if (!valid_configuration_) {
        valid_configuration_ = true;
        ROS_INFO("Trying to initialize world_model...");
        init();
    }
}

void WorldModel::ball_callback(const hlm::PixelsRelative& msg) {
    local_ball_pixel_measurements_ = msg;
    local_ball_observation_model_->set_measurement(
            local_ball_pixel_measurements_);
}

void WorldModel::obstacles_callback(const hlm::ObstaclesRelative& msg) {
    // clear the measurement vectors of the 4 filtered classes
    local_ball_pixel_measurements_.pixels.clear();
    local_obstacle_measurements_.clear();
    local_mate_measurements_.clear();
    local_opponent_measurements_.clear();
    // add the new measurements to the vectors
    for (hlm::ObstacleRelative obstacle : msg.obstacles) {
        if (obstacle.color == team_color_) {
            local_mate_measurements_.push_back(
                    PositionState(obstacle.position.x, obstacle.position.y));
        } else if (obstacle.color == opponent_color_) {
            local_opponent_measurements_.push_back(
                    PositionState(obstacle.position.x, obstacle.position.y));
        } else if (obstacle.color < 2) {  // robot unknown or unknown
            local_obstacle_measurements_.push_back(PositionStateW(
                    obstacle.position.x, obstacle.position.y, obstacle.width));
        }
    }
    // set the new measurements in the filter
    local_mate_observation_model_->set_measurement(local_mate_measurements_);
    local_opponent_observation_model_->set_measurement(
            local_opponent_measurements_);
    local_obstacle_observation_model_->set_measurement(
            local_obstacle_measurements_);
}

void WorldModel::team_data_callback(const hlm::TeamData& msg) {
    last_received_team_data_ = msg;
}
bool WorldModel::reset_filters_callback(std_srvs::Trigger::Request& req,
        std_srvs::Trigger::Response& res) {
    // pretty self-explaining... it resets all the filters
    reset_all_filters();
    res.success = true;
    res.message = "resetted all particle filters";
    return true;
}

void WorldModel::reset_all_filters() {
    ROS_INFO("Resetting all particle filters...");

    local_ball_pf_.reset(new particle_filter::ParticleFilter<PositionState>(
            config_.local_ball_particle_number, local_ball_observation_model_,
            local_ball_movement_model_));
    local_mate_pf_.reset(new particle_filter::ParticleFilter<PositionState>(
            config_.local_mate_particle_number, local_mate_observation_model_,
            local_mate_movement_model_));
    local_opponent_pf_.reset(new particle_filter::ParticleFilter<PositionState>(
            config_.local_opponent_particle_number,
            local_opponent_observation_model_, local_opponent_movement_model_));
    local_obstacle_pf_.reset(
            new particle_filter::ParticleFilter<PositionStateW>(
                    config_.local_obstacle_particle_number,
                    local_obstacle_observation_model_,
                    local_obstacle_movement_model_));

    // setting the resampling strategies
    local_ball_pf_->setResamplingStrategy(local_ball_resampling_);
    local_mate_pf_->setResamplingStrategy(local_mate_resampling_);
    local_opponent_pf_->setResamplingStrategy(local_opponent_resampling_);
    local_obstacle_pf_->setResamplingStrategy(local_obstacle_resampling_);

    // resetting the particles
    local_ball_pf_->drawAllFromDistribution(local_ball_state_distribution_);
    local_mate_pf_->drawAllFromDistribution(local_mate_state_distribution_);
    local_opponent_pf_->drawAllFromDistribution(
            local_opponent_state_distribution_);
    local_obstacle_pf_->drawAllFromDistribution(
            local_obstacle_state_distribution_);

    // setting up the distributions for the further game
    *local_ball_state_distribution_ = LocalPositionStateDistribution(
            random_number_generator_, std::make_pair(0.0, 0.0),
            std::make_pair(config_.local_ball_max_distance * 2,
                    config_.local_ball_max_distance * 2));
    *local_mate_state_distribution_ = LocalPositionStateDistribution(
            random_number_generator_, std::make_pair(0.0, 0.0),
            std::make_pair(config_.local_mate_max_distance * 2,
                    config_.local_mate_max_distance * 2));
    *local_opponent_state_distribution_ = LocalPositionStateDistribution(
            random_number_generator_, std::make_pair(0.0, 0.0),
            std::make_pair(config_.local_opponent_max_distance * 2,
                    config_.local_opponent_max_distance * 2));
    *local_obstacle_state_distribution_ = LocalPositionStateWDistribution(
            random_number_generator_, std::make_pair(0.0, 0.0),
            std::make_pair(config_.local_obstacle_max_distance * 2,
                    config_.local_obstacle_max_distance * 2),
            config_.local_obstacle_min_width, config_.local_obstacle_max_width);
}

void WorldModel::init() {
    if (!valid_configuration_) {
        ROS_ERROR(
                "You tried to initialize the world model with an invalid configuration!\n The dynamic_reconfigure_callback has to be called at least once with a valid configuration before initializing the world model!");
        return;
    }
    reset_all_filters();
}

void WorldModel::publish_particle_visualization() {
    if (!config_.debug_visualization) {
        return;
    }
    local_particles_publisher_.publish(local_ball_pf_->renderPointsMarker(
            "local_ball", config_.local_publishing_frame,
            ros::Duration(
                    1.0 / static_cast<double>(config_.publishing_frequency)),
            get_color_msg(config_.ball_marker_color)));
    local_particles_publisher_.publish(local_mate_pf_->renderPointsMarker(
            "local_mate", config_.local_publishing_frame,
            ros::Duration(
                    1.0 / static_cast<double>(config_.publishing_frequency)),
            get_color_msg(config_.mate_marker_color)));
    local_particles_publisher_.publish(local_opponent_pf_->renderPointsMarker(
            "local_opponent", config_.local_publishing_frame,
            ros::Duration(
                    1.0 / static_cast<double>(config_.publishing_frequency)),
            get_color_msg(config_.opponent_marker_color)));
    local_particles_publisher_.publish(local_obstacle_pf_->renderPointsMarker(
            "local_obstacle", config_.local_publishing_frame,
            ros::Duration(
                    1.0 / static_cast<double>(config_.publishing_frequency)),
            get_color_msg(config_.obstacle_marker_color)));
}

void WorldModel::publish_gmm_visualization(gmms::GaussianMixtureModel gmm,
        std::string n_space,
        ros::Duration lifetime) {
    local_particles_publisher_.publish(gmm.renderMarker(
            -(config_.field_width / 2), -(config_.field_height / 2),
            (config_.field_width / 2), (config_.field_height / 2), 100, n_space,
            config_.local_publishing_frame,
            lifetime));  // TODO: check whether x and y are in the right order
}

void WorldModel::exec_local_filter_step() {
    // setting the weights of the particles according to the measurements taken
    // TODO: do this only when stuff is measured
    local_ball_pf_->measure();
    local_mate_pf_->measure();
    local_opponent_pf_->measure();
    local_obstacle_pf_->measure();

    // manually clearing the list of measurements
    // TODO: just age the measurements instead of removing them
    local_ball_observation_model_->clear_measurement();
    local_mate_observation_model_->clear_measurement();
    local_opponent_observation_model_->clear_measurement();
    local_obstacle_observation_model_->clear_measurement();

    // we need to resample after every step because the world is changing
    // constantly - even if we don't move. the filters have got a resampling
    // strategy with explorers, meaning they spread some particles randomly in
    // the distribution
    local_ball_pf_->resample();
    local_mate_pf_->resample();
    local_opponent_pf_->resample();
    local_obstacle_pf_->resample();

    // diffuse the particles (add normal distributed uncertainty)
    // the .1 parameter is only there for legacy reasons and has no effect at
    // all.
    local_ball_pf_->diffuse();
    local_mate_pf_->diffuse();
    local_opponent_pf_->diffuse();
    local_obstacle_pf_->diffuse();
}

void WorldModel::publishing_timer_callback(const ros::TimerEvent&) {
    // the content of this function is what happens in a single timestep

    // executing a local filter step
    exec_local_filter_step();

    // publish the output
    publish_local_results();
}

void WorldModel::set_global_measurements(hlm::TeamData msg) {

    // cleaning up the measurement lists
    global_ball_measurements_.clear();
    global_mate_measurements_.clear();
    global_opponent_measurements_.clear();

    // iterate through all the senders stuff
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Pose buffer_pose;
    for (char sender_number = 0; sender_number < msg.robot_ids.size();
            sender_number++) {
        // the self localizatin of a robot
        mate_self_detections_[msg.robot_ids[sender_number]] =
                msg.robot_positions[sender_number];
        try {
            transformStamped = transform_buffer_.lookupTransform("map",
                    std::string("mate_") +
                            std::to_string(msg.robot_ids[sender_number]),
                    ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_WARN(
                    "Transform error when recieving message from robot id %d: %s",
                    msg.robot_ids[sender_number], ex.what());
            // ros::Duration(1.0).sleep();
            continue;
        }

        // first the own team

        // add the detecting robots own pose as mate measurement
        // TODO: does this work? or is that only one pose which is referenced?
        PositionState position_state(msg.robot_positions[sender_number].x,
                msg.robot_positions[sender_number].y);
        global_mate_measurements_.push_back(position_state);

        tf2::doTransform(pose2d_to_pose(msg.team_robot_a[sender_number].pose),
                buffer_pose, transformStamped);
        position_state =
                PositionState(buffer_pose.position.x, buffer_pose.position.y);
        global_mate_measurements_.push_back(position_state);
        tf2::doTransform(pose2d_to_pose(msg.team_robot_b[sender_number].pose),
                buffer_pose, transformStamped);
        position_state =
                PositionState(buffer_pose.position.x, buffer_pose.position.y);
        global_mate_measurements_.push_back(position_state);
        tf2::doTransform(pose2d_to_pose(msg.team_robot_c[sender_number].pose),
                buffer_pose, transformStamped);
        position_state =
                PositionState(buffer_pose.position.x, buffer_pose.position.y);
        global_mate_measurements_.push_back(position_state);
        // second, the opponents
        tf2::doTransform(
                pose2d_to_pose(msg.opponent_robot_a[sender_number].pose),
                buffer_pose, transformStamped);
        position_state =
                PositionState(buffer_pose.position.x, buffer_pose.position.y);
        global_opponent_measurements_.push_back(position_state);
        tf2::doTransform(
                pose2d_to_pose(msg.opponent_robot_b[sender_number].pose),
                buffer_pose, transformStamped);
        position_state =
                PositionState(buffer_pose.position.x, buffer_pose.position.y);
        global_opponent_measurements_.push_back(position_state);
        tf2::doTransform(
                pose2d_to_pose(msg.opponent_robot_c[sender_number].pose),
                buffer_pose, transformStamped);
        position_state =
                PositionState(buffer_pose.position.x, buffer_pose.position.y);
        global_opponent_measurements_.push_back(position_state);
        tf2::doTransform(
                pose2d_to_pose(msg.opponent_robot_d[sender_number].pose),
                buffer_pose, transformStamped);
        position_state =
                PositionState(buffer_pose.position.x, buffer_pose.position.y);
        global_opponent_measurements_.push_back(position_state);

        // now, the ball
        tf2::doTransform(pose2d_to_pose(msg.ball_relative[sender_number].pose),
                buffer_pose, transformStamped);
        position_state =
                PositionState(buffer_pose.position.x, buffer_pose.position.y);
        global_ball_measurements_.push_back(position_state);
    }

    // adding our own detections to the list


    try {
        transformStamped = transform_buffer_.lookupTransform("map",
                config_.local_publishing_frame,
                ros::Time(0));
    } catch (tf2::TransformException& ex) {
        ROS_WARN(
                "Transform error when transforming to map: %s", ex.what());
        // ros::Duration(1.0).sleep();
        return;
    }

    // adding the ball
    for (const Eigen::VectorXd mean : local_ball_gmm_.gaussianMeans()) {
        // TODO: check for certainty, use the best?
        PositionState buffer_position_state(mean.coeff(0), mean.coeff(1));
        global_ball_measurements_.push_back(buffer_position_state);
    }

    // adding mates
    for (const Eigen::VectorXd mean : local_mates_gmm_.gaussianMeans()) {
        // TODO: check for certainty
        PositionState buffer_position_state(mean.coeff(0), mean.coeff(1));
        global_mate_measurements_.push_back(buffer_position_state);
    }
    // adding opponents
    for (const Eigen::VectorXd mean : local_opponents_gmm_.gaussianMeans()) {
        // TODO: check for certainty
        PositionState buffer_position_state(mean.coeff(0), mean.coeff(1));
        global_opponent_measurements_.push_back(buffer_position_state);
    }
}

void WorldModel::publish_local_results() {
    // result acquisition and publishing

    local_ball_gmm_ = local_ball_pf_->getGMM(config_.local_ball_gmm_components,
            config_.local_ball_gmm_delta, config_.local_ball_gmm_iterations);

    local_mates_gmm_ = local_mate_pf_->getDynGMM(
            std::max(config_.local_mate_gmm_min_components,
                    local_mates_gmm_.numComponents() -
                            config_.local_mate_gmm_component_count_max_delta),
            std::min(config_.local_mate_gmm_max_components,
                    local_mates_gmm_.numComponents() +
                            config_.local_mate_gmm_component_count_max_delta),
            config_.local_mate_gmm_component_delta,
            config_.local_mate_gmm_iteration_delta,
            config_.local_mate_gmm_iterations);
    local_opponents_gmm_ = local_opponent_pf_->getDynGMM(
            std::max(config_.local_opponent_gmm_min_components,
                    local_opponents_gmm_.numComponents() -
                            config_.local_opponent_gmm_component_count_max_delta),
            std::min(config_.local_opponent_gmm_max_components,
                    local_opponents_gmm_.numComponents() +
                            config_.local_opponent_gmm_component_count_max_delta),
            config_.local_opponent_gmm_component_delta,
            config_.local_opponent_gmm_iteration_delta,
            config_.local_opponent_gmm_iterations);
    local_obstacles_gmm_ = local_obstacle_pf_->getDynGMM(
            std::max(config_.local_obstacle_gmm_min_components,
                    local_obstacles_gmm_.numComponents() -
                            config_.local_obstacle_gmm_component_count_max_delta),
            std::min(config_.local_obstacle_gmm_max_components,
                    local_obstacles_gmm_.numComponents() +
                            config_.local_obstacle_gmm_component_count_max_delta),
            config_.local_obstacle_gmm_component_delta,
            config_.local_obstacle_gmm_iteration_delta,
            config_.local_obstacle_gmm_iterations);

    hlm::Model model_msg;

    model_msg.ball = relative_gmm_to_ball_relative(
            local_ball_gmm_)[0];  // TODO: sort by confidence? currently there
                                  // is only one.
    model_msg.ball.header.stamp = ros::Time::now();  // or by input?
    model_msg.ball.header.frame_id =
            config_.local_publishing_frame;  // or by input?

    model_msg.obstacles.header =
            model_msg.ball.header;  // or this by input and the other now?
    // construct obstacles vector
    std::vector<hlm::ObstacleRelative> mates =
            relative_gmm_to_obstacle_relative(local_mates_gmm_, team_color_);
    std::vector<hlm::ObstacleRelative> opponents =
            relative_gmm_to_obstacle_relative(
                    local_opponents_gmm_, opponent_color_);
    std::vector<hlm::ObstacleRelative> obstacles =
            relative_gmm_to_obstacle_relative(
                    local_obstacles_gmm_, hlm::ObstacleRelative::UNDEFINED);

    model_msg.obstacles.obstacles = mates;
    model_msg.obstacles.obstacles.insert(
            std::end(model_msg.obstacles.obstacles), std::begin(opponents),
            std::end(opponents));
    model_msg.obstacles.obstacles.insert(
            std::end(model_msg.obstacles.obstacles), std::begin(obstacles),
            std::end(obstacles));

    local_model_publisher_.publish(model_msg);

    // the rest is visualization
    if (!config_.debug_visualization) {
        return;
    }

    // publish particles
    publish_particle_visualization();

    // publish plotted GMMs
    if (config_.local_ball_gmm_visualization) {
        publish_gmm_visualization(local_ball_gmm_, "local_ball_gmm",
                ros::Duration(1.0 / config_.publishing_frequency));
    }
    if (config_.local_mate_gmm_visualization) {
        publish_gmm_visualization(local_mates_gmm_, "local_mates_gmm",
                ros::Duration(1.0 / config_.publishing_frequency));
    }
    if (config_.local_opponent_gmm_visualization) {
        publish_gmm_visualization(local_obstacles_gmm_, "local_opponents_gmm",
                ros::Duration(1.0 / config_.publishing_frequency));
    }
    if (config_.local_obstacle_gmm_visualization) {
        publish_gmm_visualization(local_obstacles_gmm_, "local_obstacles_gmm",
                ros::Duration(1.0 / config_.publishing_frequency));
    }
}

std_msgs::ColorRGBA WorldModel::get_color_msg(int color_id) {
    float r, g, b, a = 0.8;
    switch (color_id) {
        case 0:  // White
            r = 1, g = 1, b = 1;
            break;
        case 1:  // Black
            r = 0, g = 0, b = 0;
            break;
        case 2:  // Yellow
            r = 1, g = 1, b = 0;
            break;
        case 3:  // Blue
            r = 0, g = 0, b = 1;
            break;
        case 4:  // Red
            r = 1, g = 0, b = 0;
            break;
        case 5:  // Green
            r = 0, g = 1, b = 0;
            break;
        case 6:  // Orange
            r = 1, g = .5, b = 0;
            break;
        case 7:  // Violet
            r = .8, g = 0, b = 1;
            break;
        default:
            ROS_WARN_STREAM("Got an unknown color id!");
            r = 1, g = 0, b = 0;
    }
    std_msgs::ColorRGBA color;
    color.r = r, color.g = g, color.b = b, color.a = a;
    return color;
}

std::vector<hlm::ObstacleRelative>
WorldModel::relative_gmm_to_obstacle_relative(gmms::GaussianMixtureModel gmm,
        unsigned char color) {
    std::vector<hlm::ObstacleRelative> vector;
    for (int i = 0; i < gmm.numComponents(); i++) {
        gmms::Gaussian gaussian = gmm.component(i);
        hlm::ObstacleRelative obstacle;
        obstacle.color = color;
        obstacle.position.x = gaussian.mean()[0];
        obstacle.position.y = gaussian.mean()[1];
        obstacle.confidence =
                std::min(1.0, 1 / (gaussian.covariance().coeff(0, 0) +
                                          gaussian.covariance().coeff(1,
                                                  1)));  // TODO: this in better
        vector.push_back(obstacle);
    }
    return vector;
}

std::vector<hlm::BallRelative>
WorldModel::relative_gmm_to_ball_relative(gmms::GaussianMixtureModel gmm) {
    std::vector<hlm::BallRelative> vector;
    for (int i = 0; i < gmm.numComponents(); i++) {
        gmms::Gaussian gaussian = gmm.component(i);
        hlm::BallRelative ball;
        ball.ball_relative.x = gaussian.mean()[0];
        ball.ball_relative.y = gaussian.mean()[1];
        ball.confidence =
                std::min(1.0, 1 / (gaussian.covariance().coeff(0, 0) +
                                          gaussian.covariance().coeff(1,
                                                  1)));  // TODO: this in better
        vector.push_back(ball);
    }
    return vector;
}

void WorldModel::send_mate_transforms() {
    geometry_msgs::TransformStamped transform;
    tf2::Quaternion q;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "map";
    for (char i = 1; i <=4; i++) {
        // if the element is in the dictionary
        if (mate_self_detections_.find(i) == mate_self_detections_.end()) {
            continue;
        }
        transform.child_frame_id = std::string("mate_") + std::to_string(i);
        std::pair<gmms::Gaussian, double> gaussian_distance = global_mates_gmm_.getClosestGaussian(Eigen::Vector2d(mate_self_detections_[i].x, mate_self_detections_[i].y));
        // TODO check distance in in threshold
        transform.transform.translation.x = gaussian_distance.first.mean()[0];
        transform.transform.translation.y = gaussian_distance.first.mean()[1];
        transform.transform.translation.z = 0;
        q.setRPY(0, 0, mate_self_detections_[i].theta);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        transform_broadcaster_.sendTransform(transform);
    }
}

geometry_msgs::Pose
WorldModel::pose2d_to_pose(const geometry_msgs::Pose2D pose2D) {
    geometry_msgs::Pose pose;
    pose.position.x = pose2D.x;
    pose.position.y = pose2D.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, pose2D.theta);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "world_model");
    WorldModel world_model;


    // dynamic reconfigure
    dynamic_reconfigure::Server<wm::WorldModelConfig>
            dynamic_reconfigure_server;
    dynamic_reconfigure::Server<wm::WorldModelConfig>::CallbackType f =
            boost::bind(&WorldModel::dynamic_reconfigure_callback, &world_model,
                    _1, _2);
    dynamic_reconfigure_server.setCallback(
            f);  // automatically calls the callback once


    ros::spin();
    return 0;
}
