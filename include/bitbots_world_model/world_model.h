
#ifndef WORLD_MODEL
#define WORLD_MODEL

#include <vector>
#include <memory>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <humanoid_league_msgs/ObstaclesRelative.h>
#include <humanoid_league_msgs/Model.h>
#include <dynamic_reconfigure/server.h>
#include <bitbots_world_model/WorldModelConfig.h>
#include <bitbots_world_model/ObstacleStates.h>
#include <bitbots_world_model/MovementModels.h>
#include <bitbots_world_model/ObservationModels.h>
#include <bitbots_world_model/StateDistributions.h>

#include <libPF/ParticleFilter.h>
#include <libPF/CRandomNumberGenerator.h>


namespace hlm = humanoid_league_msgs;
namespace wm = bitbots_world_model;

class WorldModel {
    public:
        WorldModel();

        void dynamic_reconfigure_callback(wm::WorldModelConfig &config, uint32_t level);
        void obstacles_callback(const hlm::ObstaclesRelative &msg);

        void init();
        void reset_all_filters();

    private:

        ros::NodeHandle nh_;

        ros::Subscriber obstacle_subscriber_;

        ros::Publisher local_model_publisher_;
        ros::Publisher global_model_publisher_;

        ros::Publisher local_obstacle_particles_publisher_;

        ros::Timer publishing_timer_;

        libPF::CRandomNumberGenerator random_number_generator_;

        // config - stuff
        bitbots_world_model::WorldModelConfig config_;
        std_msgs::ColorRGBA mate_marker_color, opponent_marker_color, obstacle_marker_color;

        int team_color_;
        int opponent_color_;

        // measurements
        std::vector<ObstacleStateW> obstacle_measurements_;
        std::vector<ObstacleState> mate_measurements_;
        std::vector<ObstacleState> opponent_measurements_;

        std::shared_ptr<LocalObstacleObservationModel> local_obstacle_observation_model_;
        std::shared_ptr<LocalObstacleObservationModel> local_robot_observation_model_;

        std::shared_ptr<LocalObstacleMovementModel> local_obstacle_movement_model_;
        std::shared_ptr<LocalObstacleMovementModel> local_robot_movement_model_;

        std::unique_ptr<libPF::ParticleFilter<ObstacleStateW>> local_obstacle_pf_;
        std::unique_ptr<libPF::ParticleFilter<ObstacleStateW>> local_mate_pf_;
        std::unique_ptr<libPF::ParticleFilter<ObstacleStateW>> local_opponent_pf_;
        std::shared_ptr<LocalObstacleStateWDistribution> local_obstacle_state_distribution_;



        bool valid_configuration_;

        void publishing_timer_callback(const ros::TimerEvent&);
        void publish_visualization();

        std_msgs::ColorRGBA get_color_msg(int color_id);

};

#endif
