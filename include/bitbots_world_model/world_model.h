
#ifndef WORLD_MODEL
#define WORLD_MODEL

#include <vector>

#include <ros/ros.h>
#include <humanoid_league_msgs/ObstaclesRelative.h>
#include <humanoid_league_msgs/Model.h>
#include <dynamic_reconfigure/server.h>
#include <bitbots_world_model/WorldModelConfig.h>
#include <bitbots_world_model/ObstacleStates.h>
#include <bitbots_world_model/MovementModels.h>
#include <bitbots_world_model/ObservationModels.h>

#include <libPF/ParticleFilter.h>


namespace hlm = humanoid_league_msgs;
namespace wm = bitbots_world_model;

class WorldModel {
    public:
        WorldModel();

        void dynamic_reconfigure_callback(wm::WorldModelConfig &config, uint32_t level);
        void obstacles_callback(const hlm::ObstaclesRelative &msg);

    private:

        ros::NodeHandle nh_;

        ros::Subscriber obstacle_subscriber_;

        ros::Publisher local_model_publisher_;
        ros::Publisher global_model_publisher_;

        unsigned int pnum = 5;

        LocalObstacleObservationModel local_obstacle_observation_model_;
        LocalObstacleObservationModel local_robot_observation_model_;

        LocalObstacleMovementModel local_obstacle_movement_model_;
        LocalObstacleMovementModel local_robot_movement_model_;

        std::unique_ptr<libPF::ParticleFilter<ObstacleStateW>> local_obstacle_pf_;
        std::unique_ptr<libPF::ParticleFilter<ObstacleStateW>> local_mate_pf_;
        std::unique_ptr<libPF::ParticleFilter<ObstacleStateW>> local_opponent_pf_;

        int team_color_;
        int opponent_color_;
};

#endif
