#include "bitbots_world_model/world_model.h"


WorldModel::WorldModel() : nh_(), valid_configuration_(false) {

    ROS_INFO("Created Bit-Bots world model");

}

void WorldModel::dynamic_reconfigure_callback(bitbots_world_model::WorldModelConfig &config, uint32_t level) {
    ROS_INFO("Dynamic reconfigure callback was called...");

    // updating topic names when neccessary
    if (config.obstacles_topic != config_.obstacles_topic) {
        obstacle_subscriber_ = nh_.subscribe(config.obstacles_topic.c_str(), 1, &WorldModel::obstacles_callback, this);
    }
    if (config.local_model_topic != config_.local_model_topic) {
        local_model_publisher_ = nh_.advertise<hlm::Model>(config.local_model_topic.c_str(), 1);
    }
    if (config.global_model_topic != config_.global_model_topic) {
        global_model_publisher_ = nh_.advertise<hlm::Model>(config.global_model_topic.c_str(), 1);
    }
    if (config.local_mate_particles_topic != config_.local_mate_particles_topic) {
        local_mate_particles_publisher_ = nh_.advertise<visualization_msgs::Marker>(config.local_mate_particles_topic.c_str(), 1);
    }
    if (config.local_opponent_particles_topic != config_.local_opponent_particles_topic) {
        local_opponent_particles_publisher_ = nh_.advertise<visualization_msgs::Marker>(config.local_opponent_particles_topic.c_str(), 1);
    }
    if (config.local_obstacle_particles_topic != config_.local_obstacle_particles_topic) {
        local_obstacle_particles_publisher_ = nh_.advertise<visualization_msgs::Marker>(config.local_obstacle_particles_topic.c_str(), 1);
    }
    if (config.reset_filters_service_name != config_.reset_filters_service_name) {
        reset_filters_service_ = nh_.advertiseService(config.reset_filters_service_name, &WorldModel::reset_filters_callback, this);
    }

    // initializing timer
    publishing_timer_ = nh_.createTimer(ros::Duration(1.0/static_cast<double>(config.publishing_frequency)), &WorldModel::publishing_timer_callback, this);

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
                    << "\" as team color. the value has to correspond with either ROBOT_MAGENTA or ROBOT_CYAN set in the ObstacleRelative message");
        }
    }

    // initializing observation models
    local_mate_observation_model_.reset(new LocalRobotObservationModel());
    local_mate_observation_model_->set_min_weight(config.local_mate_min_weight);
    local_opponent_observation_model_.reset(new LocalRobotObservationModel());
    local_opponent_observation_model_->set_min_weight(config.local_opponent_min_weight);
    local_obstacle_observation_model_.reset(new LocalObstacleObservationModel());
    local_obstacle_observation_model_->set_min_weight(config.local_obstacle_min_weight);

    // initializing movement models
    local_mate_movement_model_.reset(new LocalRobotMovementModel(random_number_generator_, config.local_mate_diffusion_x_std_dev, config.local_mate_diffusion_y_std_dev, config.local_mate_diffusion_multiplicator));
    local_opponent_movement_model_.reset(new LocalRobotMovementModel(random_number_generator_, config.local_opponent_diffusion_x_std_dev, config.local_opponent_diffusion_y_std_dev, config.local_opponent_diffusion_multiplicator));
    local_obstacle_movement_model_.reset(new LocalObstacleMovementModel(random_number_generator_, config.local_obstacle_diffusion_x_std_dev, config.local_obstacle_diffusion_y_std_dev, config.local_obstacle_diffusion_multiplicator));

    // initializing state distributions
    local_mate_state_distribution_.reset(new LocalObstacleStateDistribution(random_number_generator_, std::make_pair(config.initial_robot_x, config.initial_robot_y), std::make_pair(config.field_height, config.field_width)));
    local_opponent_state_distribution_.reset(new LocalObstacleStateDistribution(random_number_generator_, std::make_pair(config.initial_robot_x, config.initial_robot_y), std::make_pair(config.field_height, config.field_width)));
    local_obstacle_state_distribution_.reset(new LocalObstacleStateWDistribution(random_number_generator_, std::make_pair(config.initial_robot_x, config.initial_robot_y), std::make_pair(config.field_height, config.field_width), config.local_obstacle_min_width, config.local_obstacle_max_width));

    // setting particle counts
    if (config.local_obstacle_particle_number != config_.local_obstacle_particle_number || config.local_mate_particle_number != config_.local_mate_particle_number || config.local_opponent_particle_number != config_.local_opponent_particle_number) {
        ROS_INFO_STREAM("You changed the particle number to the following: \nlocal_obstacle_particle_number: " <<
                         config.local_obstacle_particle_number <<
                         "\nlocal_mate_particle_number: " <<
                         config.local_mate_particle_number <<
                         "\nlocal_opponent_particle_number: " <<
                         config.local_opponent_particle_number <<
                         "\nTo use the updated particle numbers, you need to reset the filters.");
    }

    // initializing resampling methods
    local_mate_resampling_.reset(new ImportanceResamplingWE<ObstacleState>(static_cast<int>(config.local_mate_particle_number * config.local_mate_explorer_rate), local_mate_state_distribution_));
    local_opponent_resampling_.reset(new ImportanceResamplingWE<ObstacleState>(static_cast<int>(config.local_opponent_particle_number * config.local_opponent_explorer_rate), local_opponent_state_distribution_));
    local_obstacle_resampling_.reset(new ImportanceResamplingWE<ObstacleStateW>(static_cast<int>(config.local_obstacle_particle_number * config.local_obstacle_explorer_rate), local_obstacle_state_distribution_));

    config_ = config;
    if (!valid_configuration_) {
        valid_configuration_ = true;
        ROS_INFO("Trying to initialize world_model...");
        init();
    }

    local_mate_pf_->setMarkerColor(get_color_msg(config.mate_marker_color));
    local_mate_pf_->setMarkerLifetime(ros::Duration(1.0/static_cast<double>(config.publishing_frequency)));
    local_opponent_pf_->setMarkerColor(get_color_msg(config.opponent_marker_color));
    local_opponent_pf_->setMarkerLifetime(ros::Duration(1.0/static_cast<double>(config.publishing_frequency)));
    local_obstacle_pf_->setMarkerColor(get_color_msg(config.obstacle_marker_color));
    local_obstacle_pf_->setMarkerLifetime(ros::Duration(1.0/static_cast<double>(config.publishing_frequency)));
}

void WorldModel::obstacles_callback(const hlm::ObstaclesRelative &msg) {
    // clear the measurement vectors of the 3 filtered classes
    obstacle_measurements_.clear();
    mate_measurements_.clear();
    opponent_measurements_.clear();
    for (hlm::ObstacleRelative obstacle : msg.obstacles) {
        if (obstacle.color == team_color_) {
            mate_measurements_.push_back(ObstacleState(obstacle.position.x, obstacle.position.y));
        } else if (obstacle.color == opponent_color_) {
            opponent_measurements_.push_back(ObstacleState(obstacle.position.x, obstacle.position.y));
        } else {
            // everything else is threated as an obstacle
            obstacle_measurements_.push_back(ObstacleStateW(obstacle.position.x, obstacle.position.y, obstacle.width));
        }
    }
    local_mate_observation_model_->set_measurement(mate_measurements_);
    local_opponent_observation_model_->set_measurement(opponent_measurements_);
    local_obstacle_observation_model_->set_measurement(obstacle_measurements_);
}

bool WorldModel::reset_filters_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    reset_all_filters();
    res.success = true;
    res.message = "resetted all particle filters";
    return true;
}

void WorldModel::reset_all_filters() {
    ROS_INFO("Resetting all particle filters...");

    local_mate_pf_.reset(new libPF::ParticleFilter<ObstacleState>(config_.local_mate_particle_number, local_mate_observation_model_, local_mate_movement_model_));
    local_opponent_pf_.reset(new libPF::ParticleFilter<ObstacleState>(config_.local_opponent_particle_number, local_opponent_observation_model_, local_opponent_movement_model_));
    local_obstacle_pf_.reset(new libPF::ParticleFilter<ObstacleStateW>(config_.local_obstacle_particle_number, local_obstacle_observation_model_, local_obstacle_movement_model_));

    //setting the resampling strategies
    local_mate_pf_->setResamplingStrategy(local_mate_resampling_);
    local_opponent_pf_->setResamplingStrategy(local_opponent_resampling_);
    local_obstacle_pf_->setResamplingStrategy(local_obstacle_resampling_);

    // resetting the particles
    local_mate_pf_->drawAllFromDistribution(local_mate_state_distribution_);
    local_opponent_pf_->drawAllFromDistribution(local_opponent_state_distribution_);
    local_obstacle_pf_->drawAllFromDistribution(local_obstacle_state_distribution_);

    // setting marker settings
    local_mate_pf_->setMarkerColor(get_color_msg(config_.mate_marker_color));
    local_mate_pf_->setMarkerLifetime(ros::Duration(1.0/static_cast<double>(config_.publishing_frequency)));
    local_opponent_pf_->setMarkerColor(get_color_msg(config_.opponent_marker_color));
    local_opponent_pf_->setMarkerLifetime(ros::Duration(1.0/static_cast<double>(config_.publishing_frequency)));
    local_obstacle_pf_->setMarkerColor(get_color_msg(config_.obstacle_marker_color));
    local_obstacle_pf_->setMarkerLifetime(ros::Duration(1.0/static_cast<double>(config_.publishing_frequency)));

}

void WorldModel::init() {
    if (!valid_configuration_) {
        ROS_ERROR("You tried to initialize the world model with an invalid configuration!\n The dynamic_reconfigure_callback has to be called at least once with a valid configuration before initializing the world model!");
        return;
    }
    reset_all_filters();
}

void WorldModel::publish_visualization() {
    if (!config_.debug_visualization) {
        return;
    }
    local_obstacle_particles_publisher_.publish(local_obstacle_pf_->renderMarker());
    local_mate_particles_publisher_.publish(local_mate_pf_->renderMarker());
    local_opponent_particles_publisher_.publish(local_opponent_pf_->renderMarker());
}

void WorldModel::publishing_timer_callback(const ros::TimerEvent&) {
    publish_visualization();
    // TODO: do this only when stuff is measured
    local_mate_pf_->measure();
    local_opponent_pf_->measure();
    local_obstacle_pf_->measure();
    // manually clearing the list of mearurements
    local_mate_observation_model_->clear_measurement();
    local_opponent_observation_model_->clear_measurement();
    local_obstacle_observation_model_->clear_measurement();

    // we need to resample after every step because the world is changing constantly - even if we don't move.
    local_mate_pf_->resample();
    local_opponent_pf_->resample();
    local_obstacle_pf_->resample();
    local_mate_pf_->diffuse(.1);
    local_opponent_pf_->diffuse(.1);
    local_obstacle_pf_->diffuse(.1);
}

std_msgs::ColorRGBA WorldModel::get_color_msg(int color_id) {
    double r, g, b, a = 0.8;
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
    }
    std_msgs::ColorRGBA color;
    color.r = r, color.g = g, color.b = b, color.a = a;
    return color;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "world_model");
    WorldModel world_model;


    // dynamic reconfigure
    dynamic_reconfigure::Server<wm::WorldModelConfig> dynamic_reconfigure_server;
    dynamic_reconfigure::Server<wm::WorldModelConfig>::CallbackType f = boost::bind(&WorldModel::dynamic_reconfigure_callback, &world_model, _1, _2);
    dynamic_reconfigure_server.setCallback(f); // automatically calls the callback once


    ros::spin();
    return 0;
}

