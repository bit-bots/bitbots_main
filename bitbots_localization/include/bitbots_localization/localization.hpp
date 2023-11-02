//
// Created by Judith on 08.03.19.
//

#ifndef BITBOTS_LOCALIZATION_LOCALIZATION_H
#define BITBOTS_LOCALIZATION_LOCALIZATION_H

#include <message_filters/subscriber.h>
#include <particle_filter/CRandomNumberGenerator.h>
#include <particle_filter/ParticleFilter.h>
#include <particle_filter/gaussian_mixture_model.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <bitbots_localization/MotionModel.hpp>
#include <bitbots_localization/ObservationModel.hpp>
#include <bitbots_localization/Resampling.hpp>
#include <bitbots_localization/RobotState.hpp>
#include <bitbots_localization/StateDistribution.hpp>
#include <bitbots_localization/map.hpp>
#include <bitbots_localization/srv/reset_filter.hpp>
#include <bitbots_localization/srv/set_paused.hpp>
#include <bitbots_localization/tools.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.hpp>
#include <iterator>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <soccer_vision_3d_msgs/msg/field_boundary.hpp>
#include <soccer_vision_3d_msgs/msg/goalpost_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "localization_parameters.hpp"

namespace sm = sensor_msgs;
namespace gm = geometry_msgs;
namespace pf = particle_filter;
namespace bl = bitbots_localization;
namespace sv3dm = soccer_vision_3d_msgs;

namespace bitbots_localization {
using namespace std::placeholders;

/**
 * @class Localization
 * @brief Includes the ROS interface, configuration and main loop of the Bit-Bots RoboCup localization.
 */
class Localization : public rclcpp::Node {
 public:
  explicit Localization();

  /**
   * Callback for the pause service
   * @param req Request.
   * @param res Response.
   */
  bool set_paused_callback(const std::shared_ptr<bl::srv::SetPaused::Request> req,
                           std::shared_ptr<bl::srv::SetPaused::Response> res);

  /**
   * Callback for the filter reset service
   * @param req Request.
   * @param res Response.
   */
  bool reset_filter_callback(const std::shared_ptr<bl::srv::ResetFilter::Request> req,
                             std::shared_ptr<bl::srv::ResetFilter::Response> res);

  /**
   * Checks if we have new params and if so reconfigures the filter
   * @param force_reload If true, the filter is reconfigured even if no new params are available
   */
  void updateParams(bool force_reload = false);

  /**
   * Callback for the line point cloud measurements
   * @param msg Message containing the line point cloud.
   */
  void LinePointcloudCallback(const sm::msg::PointCloud2 &msg);

  /**
   * Callback for goal posts measurements
   * @param msg Message containing the goal posts.
   */
  void GoalPostsCallback(const sv3dm::msg::GoalpostArray &msg);  // TODO

  /**
   * Callback for the relative field boundary measurements
   * @param msg Message containing the field boundary points.
   */
  void FieldboundaryCallback(const sv3dm::msg::FieldBoundary &msg);

  /**
   * Resets the state distribution of the state space
   * @param distribution The type of the distribution
   */

  void SetInitialPositionCallback(const gm::msg::PoseWithCovarianceStamped &msg);

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
  // Declare parameter listener and struct from the generate_parameter_library
  bitbots_localization::ParamListener param_listener_;
  // Data structure to hold all parameters, which is build from the schema in the 'parameters.yaml'
  bitbots_localization::Params config_;

  // Declare subscribers
  rclcpp::Subscription<sm::msg::PointCloud2>::SharedPtr line_point_cloud_subscriber_;
  rclcpp::Subscription<sv3dm::msg::GoalpostArray>::SharedPtr goal_subscriber_;
  rclcpp::Subscription<sv3dm::msg::FieldBoundary>::SharedPtr fieldboundary_subscriber_;

  rclcpp::Subscription<gm::msg::PoseWithCovarianceStamped>::SharedPtr rviz_initial_pose_subscriber_;

  // Declare publishers
  rclcpp::Publisher<gm::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_covariance_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_particles_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lines_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_ratings_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_ratings_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fieldboundary_ratings_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr field_publisher_;

  // Declare services
  rclcpp::Service<bl::srv::ResetFilter>::SharedPtr reset_service_;
  rclcpp::Service<bl::srv::SetPaused>::SharedPtr pause_service_;

  // Declare timers
  rclcpp::TimerBase::SharedPtr publishing_timer_;

  // Declare tf2 objects
  std::unique_ptr<tf2_ros::Buffer> tfBuffer;
  std::shared_ptr<tf2_ros::TransformListener> tfListener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br;

  // Declare particle filter components
  std::shared_ptr<pf::ImportanceResampling<RobotState>> resampling_;
  std::shared_ptr<RobotPoseObservationModel> robot_pose_observation_model_;
  std::shared_ptr<RobotMotionModel> robot_motion_model_;
  std::shared_ptr<particle_filter::ParticleFilter<RobotState>> robot_pf_;

  // Declare initial state distributions
  std::shared_ptr<RobotStateDistributionStartLeft> robot_state_distribution_start_left_;
  std::shared_ptr<RobotStateDistributionStartRight> robot_state_distribution_start_right_;
  std::shared_ptr<RobotStateDistributionRightHalf> robot_state_distribution_right_half_;
  std::shared_ptr<RobotStateDistributionLeftHalf> robot_state_distribution_left_half_;
  std::shared_ptr<RobotStateDistributionPosition> robot_state_distribution_position_;
  std::shared_ptr<RobotStateDistributionPose> robot_state_distribution_pose_;

  // Declare filter estimate
  RobotState estimate_;
  std::vector<double> estimate_cov_;

  // Declare input message buffers
  sm::msg::PointCloud2 line_pointcloud_relative_;
  sv3dm::msg::GoalpostArray goal_posts_relative_;
  sv3dm::msg::FieldBoundary fieldboundary_relative_;

  // Declare time stamps
  rclcpp::Time last_stamp_lines = rclcpp::Time(0);
  rclcpp::Time last_stamp_goals = rclcpp::Time(0);
  rclcpp::Time last_stamp_fb_points = rclcpp::Time(0);
  rclcpp::Time last_stamp_all_measurements = rclcpp::Time(0);
  builtin_interfaces::msg::Time map_odom_tf_last_published_time_ =
      builtin_interfaces::msg::Time(rclcpp::Time(0, 0, RCL_ROS_TIME));
  builtin_interfaces::msg::Time localization_tf_last_published_time_ =
      builtin_interfaces::msg::Time(rclcpp::Time(0, 0, RCL_ROS_TIME));

  // Declare robot movement variables
  geometry_msgs::msg::Vector3 linear_movement_;
  geometry_msgs::msg::Vector3 rotational_movement_;

  // Keep track of the odometry transform in the last step
  geometry_msgs::msg::TransformStamped previousOdomTransform_;

  // Flag that checks if the robot is moving
  bool robot_moved = false;

  // Keep track of the number of filter steps
  int timer_callback_count_ = 0;

  // Maps for the different measurement classes
  std::shared_ptr<Map> lines_;
  std::shared_ptr<Map> goals_;
  std::shared_ptr<Map> field_boundary_;

  // RNG that is used for the different sampling steps
  particle_filter::CRandomNumberGenerator random_number_generator_;

  /**
   * Runs the filter for one step
   */
  void run_filter_one_step();

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
   * Publishes the rating visualizations for each measurement class
   */
  void publish_ratings();

  /**
   * Publishes the rating visualizations for a arbitrary measurement class
   * @param measurements all measurements of the measurement class
   * @param scale scale of the markers
   * @param name name of the class
   * @param map map for this class
   * @param publisher ros publisher for the type visualization_msgs::msg::Marker
   */
  void publish_debug_rating(std::vector<std::pair<double, double>> measurements, double scale, const char name[24],
                            std::shared_ptr<Map> map,
                            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &publisher);

  /**
   * Updates the measurements for all classes
   */
  void updateMeasurements();

  /**
   * Gets the and convert motion / odometry information
   */
  void getMotion();
};
};  // namespace bitbots_localization

#endif  // BITBOTS_LOCALIZATION_LOCALIZATION_H
