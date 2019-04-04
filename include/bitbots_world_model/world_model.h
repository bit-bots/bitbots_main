
#ifndef WORLD_MODEL
#define WORLD_MODEL

#include <vector>
#include <map>
#include <memory>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <humanoid_league_msgs/ObstacleRelative.h>
#include <humanoid_league_msgs/ObstaclesRelative.h>
#include <humanoid_league_msgs/BallRelative.h>
#include <humanoid_league_msgs/Model.h>
#include <humanoid_league_msgs/PixelsRelative.h>
#include <humanoid_league_msgs/PixelRelative.h>
#include <humanoid_league_msgs/TeamData.h>
#include <dynamic_reconfigure/server.h>
#include <bitbots_world_model/WorldModelConfig.h>
#include <bitbots_world_model/ObstacleStates.h>
#include <bitbots_world_model/MovementModels.h>
#include <bitbots_world_model/ObservationModels.h>
#include <bitbots_world_model/StateDistributions.h>
#include <bitbots_world_model/Resampling.h>

#include <particle_filter/ParticleFilter.h>
#include <particle_filter/gaussian_mixture_model.h>
#include <particle_filter/CRandomNumberGenerator.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


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

    bool reset_filters_callback(std_srvs::Trigger::Request& req,
            std_srvs::Trigger::Response& res);

    void
    dynamic_reconfigure_callback(wm::WorldModelConfig& config, uint32_t level);
    void ball_callback(const hlm::PixelsRelative& msg);
    void obstacles_callback(const hlm::ObstaclesRelative& msg);
    void team_data_callback(const hlm::TeamData& msg);

    void init();
    void reset_all_filters();

private:
    ros::NodeHandle nh_;

    ros::Subscriber obstacle_subscriber_;
    ros::Subscriber ball_subscriber_;
    ros::Subscriber team_data_subscriber_;

    ros::Publisher local_model_publisher_;
    ros::Publisher global_model_publisher_;

    ros::Publisher local_particles_publisher_;

    ros::ServiceServer reset_filters_service_;

    ros::Timer publishing_timer_;

    tf2_ros::TransformBroadcaster transform_broadcaster_;
    tf2_ros::Buffer transform_buffer_;
    tf2_ros::TransformListener transform_listener_;

    particle_filter::CRandomNumberGenerator random_number_generator_;

    // config - stuff
    bitbots_world_model::WorldModelConfig config_;
    std_msgs::ColorRGBA ball_marker_color, mate_marker_color,
            opponent_marker_color, obstacle_marker_color;

    unsigned char team_color_;
    unsigned char opponent_color_;

    // local measurements
    hlm::PixelsRelative local_ball_pixel_measurements_;
    std::vector<PositionState> local_mate_measurements_;
    std::vector<PositionState> local_opponent_measurements_;
    std::vector<PositionStateW>
            local_obstacle_measurements_;  // TODO: handle these?

    // last local results
    // TODO

    // last team_data message
    hlm::TeamData last_received_team_data_;

    // global measurements
    std::vector<PositionState> global_ball_measurements_;
    std::vector<PositionState> global_mate_measurements_;
    std::vector<PositionState> global_opponent_measurements_;
    std::vector<PositionStateW> global_obstacle_measurements_;

    // resampling strategies
    // local
    std::shared_ptr<ImportanceResamplingWE<PositionState>>
            local_ball_resampling_;
    std::shared_ptr<ImportanceResamplingWE<PositionState>>
            local_mate_resampling_;
    std::shared_ptr<ImportanceResamplingWE<PositionState>>
            local_opponent_resampling_;
    std::shared_ptr<ImportanceResamplingWE<PositionStateW>>
            local_obstacle_resampling_;
    // global
    std::shared_ptr<ImportanceResamplingWE<PositionState>>
            global_ball_resampling_;
    std::shared_ptr<ImportanceResamplingWE<PositionState>>
            global_mate_resampling_;
    std::shared_ptr<ImportanceResamplingWE<PositionState>>
            global_opponent_resampling_;

    // observation models
    // local
    std::shared_ptr<LocalFcnnObservationModel> local_ball_observation_model_;
    std::shared_ptr<LocalRobotObservationModel> local_mate_observation_model_;
    std::shared_ptr<LocalRobotObservationModel>
            local_opponent_observation_model_;
    std::shared_ptr<LocalObstacleObservationModel>
            local_obstacle_observation_model_;
    // global
    std::shared_ptr<GlobalBallObservationModel> global_ball_observation_model_;
    std::shared_ptr<GlobalRobotObservationModel> global_mate_observation_model_;
    std::shared_ptr<GlobalRobotObservationModel>
            global_opponent_observation_model_;

    // movement models
    // local
    std::shared_ptr<LocalRobotMovementModel> local_ball_movement_model_;
    std::shared_ptr<LocalRobotMovementModel> local_mate_movement_model_;
    std::shared_ptr<LocalRobotMovementModel> local_opponent_movement_model_;
    std::shared_ptr<LocalObstacleMovementModel> local_obstacle_movement_model_;
    // global
    std::shared_ptr<GlobalBallMovementModel> global_ball_movement_model_;
    std::shared_ptr<GlobalRobotMovementModel> global_mate_movement_model_;
    std::shared_ptr<GlobalRobotMovementModel> global_opponent_movement_model_;

    // state distributions
    // local
    std::shared_ptr<LocalPositionStateDistribution>
            local_ball_state_distribution_;
    std::shared_ptr<LocalPositionStateDistribution>
            local_mate_state_distribution_;
    std::shared_ptr<LocalPositionStateDistribution>
            local_opponent_state_distribution_;
    std::shared_ptr<LocalPositionStateWDistribution>
            local_obstacle_state_distribution_;
    // global
    std::shared_ptr<GlobalPositionStateDistribution>
            global_ball_state_distribution_;
    std::shared_ptr<GlobalPositionStateDistribution>
            global_mate_state_distribution_;
    std::shared_ptr<GlobalPositionStateDistribution>
            global_opponent_state_distribution_;

    // particle filters
    // local
    std::shared_ptr<particle_filter::ParticleFilter<PositionState>>
            local_ball_pf_;
    std::shared_ptr<particle_filter::ParticleFilter<PositionState>>
            local_mate_pf_;
    std::shared_ptr<particle_filter::ParticleFilter<PositionState>>
            local_opponent_pf_;
    std::shared_ptr<particle_filter::ParticleFilter<PositionStateW>>
            local_obstacle_pf_;
    // global
    std::shared_ptr<particle_filter::ParticleFilter<PositionState>>
            global_ball_pf_;
    std::shared_ptr<particle_filter::ParticleFilter<PositionState>>
            global_mate_pf_;
    std::shared_ptr<particle_filter::ParticleFilter<PositionState>>
            global_opponent_pf_;

    // saving the gmms after each filter step
    // local
    gmms::GaussianMixtureModel local_ball_gmm_;
    gmms::GaussianMixtureModel local_mates_gmm_;
    gmms::GaussianMixtureModel local_opponents_gmm_;
    gmms::GaussianMixtureModel local_obstacles_gmm_;
    // global
    gmms::GaussianMixtureModel global_ball_gmm_;
    gmms::GaussianMixtureModel global_mates_gmm_;
    gmms::GaussianMixtureModel global_opponents_gmm_;

    // the self detections. they are overwritten by detections in the global
    // filter. Thus, the values are as current as possible.
    std::map<char, geometry_msgs::Pose2D> mate_self_detections_;

    bool valid_configuration_;

    std::vector<hlm::ObstacleRelative> relative_gmm_to_obstacle_relative(
            gmms::GaussianMixtureModel gmm, unsigned char color);
    std::vector<hlm::BallRelative>
    relative_gmm_to_ball_relative(gmms::GaussianMixtureModel gmm);

    void publishing_timer_callback(const ros::TimerEvent&);
    void publish_particle_visualization();
    void publish_gmm_visualization(gmms::GaussianMixtureModel gmm,
            std::string n_space, ros::Duration lifetime);
    void send_mate_transforms();
    geometry_msgs::Pose pose2d_to_pose(const geometry_msgs::Pose2D pose2D);

    std_msgs::ColorRGBA get_color_msg(int color_id);
    void publish_local_results();
    void exec_local_filter_step();
};

#endif
