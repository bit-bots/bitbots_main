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
    if (config.local_obstacle_particles_topic != config_.local_obstacle_particles_topic) {
        local_obstacle_particles_publisher_ = nh_.advertise<visualization_msgs::Marker>(config.local_obstacle_particles_topic.c_str(), 1);
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
    local_obstacle_observation_model_.reset(new LocalObstacleObservationModel());

    // initializing movement models
    local_obstacle_movement_model_.reset(new LocalObstacleMovementModel(random_number_generator_, config.local_obstacle_diffusion_x_std_dev, config.local_obstacle_diffusion_y_std_dev, config.local_obstacle_diffusion_multiplicator));

    // initializing state distributions
    local_obstacle_state_distribution_.reset(new LocalObstacleStateWDistribution(random_number_generator_, std::make_pair(config.initial_robot_x, config.initial_robot_y), std::make_pair(config.field_height, config.field_width), config.local_obstacle_min_width, config.local_obstacle_max_width));

    // initializing particle filters
    //
    // using reset because they are unique pointers
    if (config.local_obstacle_particle_number != config_.local_obstacle_particle_number || config.local_mate_particle_number != config_.local_mate_particle_number || config.local_opponent_particle_number != config_.local_opponent_particle_number) {
        ROS_INFO_STREAM("You changed the particle number to the following: \nlocal_obstacle_particle_number: " <<
                         config.local_obstacle_particle_number <<
                         "\nlocal_mate_particle_number: " <<
                         config.local_mate_particle_number <<
                         "\nlocal_opponent_particle_number: " <<
                         config.local_opponent_particle_number <<
                         "\nTo use the updated particle numbers, you need to reset the filters.");
    }
    // local_mate_pf_.reset(new libPF::ParticleFilter<ObstacleStateW>(config.local_mate_particle_number, &local_robot_observation_model_, &local_robot_movement_model_));
    // local_opponent_pf_.reset(new libPF::ParticleFilter<ObstacleStateW>(config.local_opponent_particle_number, &local_robot_observation_model_, &local_robot_movement_model_));

    config_ = config;
    if (!valid_configuration_) {
        valid_configuration_ = true;
        ROS_INFO("Trying to initialize world_model...");
        init();
    }
}

void WorldModel::obstacles_callback(const hlm::ObstaclesRelative &msg) {
    obstacle_measurements_.clear();
    for (hlm::ObstacleRelative obstacle : msg.obstacles) {
        if (obstacle.color == hlm::ObstacleRelative::UNDEFINED) {  // make this the else-case
            obstacle_measurements_.push_back(ObstacleStateW(obstacle.position.x, obstacle.position.z, obstacle.width));
        }
    }
    local_obstacle_observation_model_->set_measurement(obstacle_measurements_);
}

void WorldModel::reset_all_filters() {
    ROS_INFO("Resetting all particle filters...");

    local_obstacle_pf_.reset(new libPF::ParticleFilter<ObstacleStateW>(config_.local_obstacle_particle_number, local_obstacle_observation_model_, local_obstacle_movement_model_));

    // resetting the particles
    local_obstacle_pf_->drawAllFromDistribution(local_obstacle_state_distribution_);


}

void WorldModel::init() {
    if (!valid_configuration_) {
        ROS_ERROR("You tried to initialize the world model with an invalid configuration!\n The dynamic_reconfigure_callback has to be called at least once with a valif configuration before initializing the world model!");
        return;
    }
    reset_all_filters();
}

void WorldModel::publish_visualization() {
    if (!config_.debug_visualization) {
        return;
    }
    local_obstacle_particles_publisher_.publish(local_obstacle_pf_->renderMarker());
}

void WorldModel::publishing_timer_callback(const ros::TimerEvent&) {
    publish_visualization();
    // TODO: do this only when stuff is measured
    local_obstacle_pf_->measure();
    local_obstacle_pf_->resample();
    local_obstacle_pf_->diffuse(.1);
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

