#include "bitbots_world_model/world_model.h"


WorldModel::WorldModel() : nh_() {
    // server(nh_);
    // server.setCallback(&WorldModel::dynamic_reconfigure_callback);

    ROS_INFO("Initialized Bit-Bots world model");
}

void WorldModel::dynamic_reconfigure_callback(bitbots_world_model::WorldModelConfig &config, uint32_t level) {
    obstacle_subscriber_ = nh_.subscribe(config.obstacles_topic.c_str(), 1, &WorldModel::obstacles_callback, this);
    local_model_publisher_ = nh_.advertise<hlm::Model>(config.local_model_topic.c_str(), 1);
    global_model_publisher_ = nh_.advertise<hlm::Model>(config.global_model_topic.c_str(), 1);

    // team color
    if (config.team_color != team_color_) {
        if (config.team_color == hlm::ObstacleRelative::ROBOT_MAGENTA) {
            team_color_ = config.team_color;
            ROS_INFO("Switched team color to red");
        } else if (config.team_color == hlm::ObstacleRelative::ROBOT_CYAN) {
            team_color_ = config.team_color;
            ROS_INFO("Switched team color to blue");
        } else {
            ROS_INFO_STREAM(
                    "Could not set the input \""
                    << config.team_color
                    << "\" as team color. the value has to correspond with either ROBOT_MAGENTA or ROBOT_CYAN set in the ObstacleRelative message");
        }
    }

}

void WorldModel::obstacles_callback(const hlm::ObstaclesRelative &msg) {}

int main(int argc, char **argv) {
    ros::init(argc, argv, "world_model");
    return 0;
}