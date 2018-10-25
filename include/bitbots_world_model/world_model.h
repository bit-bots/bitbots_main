
#ifndef WORLD_MODEL
#define WORLD_MODEL

#include <vector>
#include <memory>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <humanoid_league_msgs/ObstaclesRelative.h>
#include <humanoid_league_msgs/Model.h>
#include <dynamic_reconfigure/server.h>
#include <bitbots_world_model/WorldModelConfig.h>
#include <bitbots_world_model/ObstacleStates.h>
#include <bitbots_world_model/MovementModels.h>
#include <bitbots_world_model/ObservationModels.h>
#include <bitbots_world_model/StateDistributions.h>
#include <bitbots_world_model/Resampling.h>

#include <libPF/ParticleFilter.h>
#include <libPF/CRandomNumberGenerator.h>


namespace hlm = humanoid_league_msgs;
namespace wm = bitbots_world_model;

class WorldModel {
    public:
        WorldModel();

        // Needed services:
        // - reset all filters
        // - reset a specific filter?
        // - start/stop filtering
        // - start/stop sending?

        bool reset_filters_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

        void dynamic_reconfigure_callback(wm::WorldModelConfig &config, uint32_t level);
        void obstacles_callback(const hlm::ObstaclesRelative &msg);

        void init();
        void reset_all_filters();

    private:

        ros::NodeHandle nh_;

        ros::Subscriber obstacle_subscriber_;

        ros::Publisher local_model_publisher_;
        ros::Publisher global_model_publisher_;

        ros::Publisher local_mate_particles_publisher_;
        ros::Publisher local_opponent_particles_publisher_;
        ros::Publisher local_obstacle_particles_publisher_;

        ros::ServiceServer reset_filters_service_;

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

        std::shared_ptr<ImportanceResamplingWE<ObstacleState>> local_mate_resampling_;
        std::shared_ptr<ImportanceResamplingWE<ObstacleState>> local_opponent_resampling_;
        std::shared_ptr<ImportanceResamplingWE<ObstacleStateW>> local_obstacle_resampling_;

        std::shared_ptr<LocalRobotObservationModel> local_mate_observation_model_;
        std::shared_ptr<LocalRobotObservationModel> local_opponent_observation_model_;
        std::shared_ptr<LocalObstacleObservationModel> local_obstacle_observation_model_;

        std::shared_ptr<LocalRobotMovementModel> local_mate_movement_model_;
        std::shared_ptr<LocalRobotMovementModel> local_opponent_movement_model_;
        std::shared_ptr<LocalObstacleMovementModel> local_obstacle_movement_model_;

        std::shared_ptr<LocalObstacleStateDistribution> local_mate_state_distribution_;
        std::shared_ptr<LocalObstacleStateDistribution> local_opponent_state_distribution_;
        std::shared_ptr<LocalObstacleStateWDistribution> local_obstacle_state_distribution_;

        std::shared_ptr<libPF::ParticleFilter<ObstacleStateW>> local_obstacle_pf_;
        std::shared_ptr<libPF::ParticleFilter<ObstacleState>> local_mate_pf_;
        std::shared_ptr<libPF::ParticleFilter<ObstacleState>> local_opponent_pf_;


        bool valid_configuration_;

        void publishing_timer_callback(const ros::TimerEvent&);
        void publish_visualization();

        std_msgs::ColorRGBA get_color_msg(int color_id);

};

#endif
