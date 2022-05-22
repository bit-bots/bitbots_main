//
// Created by judith on 08.03.19.
//

#ifndef BITBOTS_LOCALIZATION_LOCALIZATION_H
#define BITBOTS_LOCALIZATION_LOCALIZATION_H

#include <vector>
#include <memory>
#include <iterator>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Core>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <soccer_vision_3d_msgs/msg/goalpost_array.hpp>
#include <soccer_vision_3d_msgs/msg/field_boundary.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

#include <particle_filter/ParticleFilter.h>
#include <particle_filter/gaussian_mixture_model.h>
#include <particle_filter/CRandomNumberGenerator.h>

#include <bitbots_localization/config.h>
#include <bitbots_localization/map.h>
#include <bitbots_localization/ObservationModel.h>
#include <bitbots_localization/MotionModel.h>
#include <bitbots_localization/StateDistribution.h>
#include <bitbots_localization/Resampling.h>
#include <bitbots_localization/RobotState.h>

#include <bitbots_localization/srv/reset_filter.hpp>
#include <bitbots_localization/srv/set_paused.hpp>
#include <bitbots_localization/tools.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace sm = sensor_msgs;
namespace gm = geometry_msgs;
namespace pf = particle_filter;
namespace bl = bitbots_localization;
namespace sv3dm = soccer_vision_3d_msgs;


namespace bitbots_localization {
/**
* @class Localization
* @brief Includes the ROS interface, configuration and main loop of the Bit-Bots RoboCup localization.
*/
class Localization : public rclcpp::Node {

 public:
  explicit Localization(std::string ns, std::vector<rclcpp::Parameter> parameters = {});

  /**
   * Callback for the pause service
   * @param req Request.
   * @param res Response.
   */
  bool set_paused_callback( bl::srv::SetPaused::Request &req,
                            bl::srv::SetPaused::Response &res);

  /**
   * Callback for the filter reset service
   * @param req Request.
   * @param res Response.
   */
  bool reset_filter_callback(bl::srv::ResetFilter::Request &req,
                             bl::srv::ResetFilter::Response &res);

  /**
   * Callback for ros dynamic reconfigure
   * @param config the updated config.
   * @param config_level Not used.
   */
  void dynamic_reconfigure_callback(const std::vector<rclcpp::Parameter> &parameters);

  /**
   * Callback for the line point cloud messurements
   * @param msg Message containing the line point cloud.
   */
  void LinePointcloudCallback(const sm::msg::PointCloud2 &msg);

  /**
   * Callback for goal posts messurements
   * @param msg Message containing the goal posts.
   */
  void GoalPostsCallback(const sv3dm::msg::GoalpostArray &msg); //TODO

  /**
   * Callback for the relative field boundary messurements
   * @param msg Message containing the field boundary points.
   */
  void FieldboundaryCallback(const sv3dm::msg::FieldBoundary &msg);

  /**
   * Resets the state distribution of the state space
   * @param distribution The type of the distribution
   */
  void reset_filter(int distribution);

  /**
   * Resets the state distribution of the state space
   * @param distribution The type of the distribution
   * @param x Position of the new state distribution
   * @param y Position of the new state distribution
   */
  void reset_filter(int distribution, double x, double y);

  /**
   * Resets the state distribution of the state space
   * @param distribution The type of the distribution
   * @param x Position of the new state distribution
   * @param y Position of the new state distribution
   * @param angle Angle in which the particle distribution is centered
   */
  void reset_filter(int distribution, double x, double y, double angle);

 private:
  rclcpp::Subscription<sm::msg::PointCloud2>::SharedPtr line_point_cloud_subscriber_;
  rclcpp::Subscription<sv3dm::msg::GoalpostArray>::SharedPtr goal_subscriber_;
  rclcpp::Subscription<sv3dm::msg::FieldBoundary>::SharedPtr fieldboundary_subscriber_;

  rclcpp::Publisher<gm::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_covariance_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_particles_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lines_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_ratings_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_ratings_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fieldboundary_ratings_publisher_;

  rclcpp::Service<bl::srv::ResetFilter>::SharedPtr reset_service_;
  rclcpp::Service<bl::srv::SetPaused>::SharedPtr pause_service_;
  rclcpp::TimerBase::SharedPtr publishing_timer_;
  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br;

  std::shared_ptr<pf::ImportanceResampling<RobotState>> resampling_;
  std::shared_ptr<RobotPoseObservationModel> robot_pose_observation_model_;
  std::shared_ptr<RobotMotionModel> robot_motion_model_;
  //std::shared_ptr<RobotStateDistribution> robot_state_distribution_;
  std::shared_ptr<RobotStateDistributionStartLeft> robot_state_distribution_start_left_;
  std::shared_ptr<RobotStateDistributionStartRight> robot_state_distribution_start_right_;
  std::shared_ptr<RobotStateDistributionRightHalf> robot_state_distribution_right_half_;
  std::shared_ptr<RobotStateDistributionLeftHalf> robot_state_distribution_left_half_;
  std::shared_ptr<RobotStateDistributionPosition> robot_state_distribution_position_;
  std::shared_ptr<RobotStateDistributionPose> robot_state_distribution_pose_;
  std::shared_ptr<particle_filter::ParticleFilter<RobotState>> robot_pf_;
  RobotState estimate_;
  std::vector<double> estimate_cov_;

  bool resampled_ = false;

  sm::msg::PointCloud2 line_pointcloud_relative_;
  sv3dm::msg::GoalpostArray goal_posts_relative_;
  sv3dm::msg::FieldBoundary fieldboundary_relative_;
  sm::msg::CameraInfo cam_info_;

  rclcpp::Time last_stamp_lines = rclcpp::Time(0);
  rclcpp::Time last_stamp_lines_pc = rclcpp::Time(0);
  rclcpp::Time last_stamp_goals = rclcpp::Time(0);
  rclcpp::Time last_stamp_fb_points = rclcpp::Time(0);
  rclcpp::Time localization_tf_last_published_time_ = rclcpp::Time(0);
  rclcpp::Time map_odom_tf_last_published_time_ = rclcpp::Time(0);

  std::string odom_frame_, base_footprint_frame_, map_frame_, publishing_frame_;

  /**
   * Runs the filter for one step
   */
  void run_filter_one_step();

  std::shared_ptr<Map> lines_;
  std::shared_ptr<Map> goals_;
  std::shared_ptr<Map> field_boundary_;
  std::shared_ptr<Map> corner_;
  std::shared_ptr<Map> t_crossings_map_;
  std::shared_ptr<Map> crosses_map_;

  gmms::GaussianMixtureModel pose_gmm_;
  particle_filter::CRandomNumberGenerator random_number_generator_;
  std::shared_ptr<bl::Config> config_;
  std_msgs::ColorRGBA marker_color;
  bool first_configuration_ = true;

  /**
   * Publishes the position as a transform
   */
  void publish_transforms();

  /**
   * Publishes the position as a message
   */
  void publish_pose_with_covariance();

  /**
   * Debug publisher
   */
  void publish_debug();

  /**
   * Publishes the visualization markers for each particle
   */
  void publish_particle_markers();

  /**
   * Publishes the rating visualizations for each meassurement class
   */
  void publish_ratings();

  /**
   * Publishes the rating visualizations for a abitray messurement class
   * @param measurements all measurements of the messurement class
   * @param scale scale of the markers
   * @param name name of the class
   * @param map map for this class
   * @param publisher ros publisher for the type visualization_msgs::msg::Marker
   */
  void publish_debug_rating(
    std::vector<std::pair<double, double>>    measurements,
    double scale,
    const char name[24],
    std::shared_ptr<Map> map,
    rclcpp::Publisher<visualization_msgs::msg::Marker> &publisher);

  /**
   * Updates the messurements for all classes
   */
  void updateMeasurements();

  /**
   * Gets the and convert motion / odometry information
   */
  void getMotion();

  geometry_msgs::msg::TransformStamped transformOdomBaseLink;
  bool initialization = true;

  geometry_msgs::msg::Vector3 linear_movement_;
  geometry_msgs::msg::Vector3 rotational_movement_;
  geometry_msgs::msg::TransformStamped previousOdomTransform_;
  bool new_linepoints_ = false;
  bool robot_moved = false;
  int timer_callback_count_ = 0;
};
};

#endif //BITBOTS_LOCALIZATION_LOCALIZATION_H