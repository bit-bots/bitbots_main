#include "bitbots_world_model/world_model.h"


WorldModel::WorldModel() : nh_() {
    // server(nh_);
    // server.setCallback(&WorldModel::dynamic_reconfigure_callback);

    ROS_INFO("Initialized Bit-Bots world model");
}

void WorldModel::dynamic_reconfigure_callback(bitbots_world_model::WorldModelConfig &config, uint32_t level) {
    obstacle_subscriber_ = nh_.subscribe(config.obstacles_topic.c_str(), 1, &WorldModel::obstacles_callback, this);
}

void WorldModel::obstacles_callback(const hlm::ObstaclesRelative &msg) {}

int main(int argc, char **argv) {
    ros::init(argc, argv, "world_model");
    return 0;
}