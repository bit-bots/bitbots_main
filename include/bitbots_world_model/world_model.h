
#ifndef WORLD_MODEL
#define WORLD_MODEL

#include <vector>

#include <ros/ros.h>
#include <humanoid_league_msgs/ObstaclesRelative.h>
#include <dynamic_reconfigure/server.h>
#include <bitbots_world_model/WorldModelConfig.h>

namespace hlm = humanoid_league_msgs;

class WorldModel {
    public:
        WorldModel();

        void dynamic_reconfigure_callback(bitbots_world_model::WorldModelConfig &config, uint32_t level);
        void obstacles_callback(const hlm::ObstaclesRelative &msg);

    private:

        ros::NodeHandle nh_;

        ros::Subscriber obstacle_subscriber_;

        ros::Publisher model_publisher_;

        dynamic_reconfigure::Server<bitbots_world_model::WorldModelConfig> server;
        dynamic_reconfigure::Server<bitbots_world_model::WorldModelConfig>::CallbackType f;

        std::vector<hlm::ObstaclesRelative> obstacles_;
        std::vector<hlm::ObstaclesRelative> mates_;
        std::vector<hlm::ObstaclesRelative> opponents_;
};

#endif