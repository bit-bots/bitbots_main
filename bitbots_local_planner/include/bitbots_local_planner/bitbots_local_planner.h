#ifndef bitbots_local_planner_FTC_PLANNER_H_
#define bitbots_local_planner_FTC_PLANNER_H_

#include <nav_msgs/msg/odometry.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <nav2_core/controller.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <string>
#include <angles/angles.h>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace bitbots_local_planner {

class BBPlanner : public nav2_core::Controller {

 public:
  BBPlanner();

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Method to cleanup resources.
   */
  void cleanup() override;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  void activate() override;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  virtual void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  void motionOdomCB(const nav_msgs::msg::Odometry::SharedPtr msg);

  ~BBPlanner() override;

 private:

  geometry_msgs::msg::Pose getXPose(
    const std::string &global_frame,
    int plan_point);

  /**
  *@brief Publish the global plan for visualization.
  */
  void publishPlan();

  // Holds reference for the controller node
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

  // Holds a reference to the logger
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // used for transformation
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // global plan which we run along
  std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
  // transformed global plan in global frame with only the points with are needed for calculation (max_points)
  std::vector<geometry_msgs::msg::PoseStamped> transformed_global_plan_;
  // check if plan first at first time
  bool first_setPlan_;
  // last point of the global plan in global frame
  geometry_msgs::msg::Pose goal_pose_;
  // true if the robot should rotate to gobal plan if new global goal set
  geometry_msgs::msg::Pose old_goal_pose_;
  // The final pose
  geometry_msgs::msg::Pose end_pose_;
  // Publisher where the local plan for visulatation is published
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_publisher_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  nav_msgs::msg::Odometry::SharedPtr motion_odom_;

  // Parameters
  double config_translation_slow_down_factor;
  double config_max_vel_x;
  double config_min_vel_x;
  double config_max_vel_y;
  double config_smoothing_k;
  double config_orient_to_goal_distance;
  double config_rotation_slow_down_factor;
  double config_max_rotation_vel;
  int config_carrot_distance;
};
}
#endif
