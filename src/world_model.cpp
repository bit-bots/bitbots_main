#include "bitbots_world_model/world_model.h"


WorldModel::WorldModel() : nh_() {

    ROS_INFO("Initialized Bit-Bots world model");
}

void WorldModel::dynamic_reconfigure_callback(bitbots_world_model::WorldModelConfig &config, uint32_t level) {
    ROS_INFO("Dynamic reconfigure callback was called...");

    if (config.obstacles_topic != config_.obstacles_topic) {
        obstacle_subscriber_ = nh_.subscribe(config.obstacles_topic.c_str(), 1, &WorldModel::obstacles_callback, this);
    }
    if (config.local_model_topic != config_.local_model_topic) {
        local_model_publisher_ = nh_.advertise<hlm::Model>(config.local_model_topic.c_str(), 1);
    }
    if (config.global_model_topic != config_.global_model_topic) {
        global_model_publisher_ = nh_.advertise<hlm::Model>(config.global_model_topic.c_str(), 1);
    }

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

    // initializing particle filters
    //
    // using reset because they are unique pointers
    local_obstacle_pf_.reset(new libPF::ParticleFilter<ObstacleStateW>(pnum, &local_obstacle_observation_model_, &local_obstacle_movement_model_));
    local_mate_pf_.reset(new libPF::ParticleFilter<ObstacleStateW>(pnum, &local_robot_observation_model_, &local_robot_movement_model_));
    local_opponent_pf_.reset(new libPF::ParticleFilter<ObstacleStateW>(pnum, &local_robot_observation_model_, &local_robot_movement_model_));
    config_ = config;
}

void WorldModel::obstacles_callback(const hlm::ObstaclesRelative &msg) {
    for (hlm::ObstacleRelative obstacle : msg.obstacles) {
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "world_model");
    WorldModel world_model;


    // dynamic reconfigure
    dynamic_reconfigure::Server<wm::WorldModelConfig> dynamic_reconfigure_server;
    dynamic_reconfigure::Server<wm::WorldModelConfig>::CallbackType f = boost::bind(&WorldModel::dynamic_reconfigure_callback, &world_model, _1, _2);
    dynamic_reconfigure_server.setCallback(f); // automatically calls the callback

    ros::spin();
    return 0;
}

