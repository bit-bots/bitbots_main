#include <bitbots_local_planner/bitbots_local_planner.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(bitbots_local_planner::BBPlanner, nav_core::BaseLocalPlanner)

namespace bitbots_local_planner {

BBPlanner::BBPlanner() {
}

void BBPlanner::initialize(std::string name, tf2_ros::Buffer *tf_buffer, costmap_2d::Costmap2DROS *costmap_ros) {
  ros::NodeHandle private_nh("~/" + name);
  local_plan_publisher_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
  tf_buffer_ = tf_buffer;
  first_setPlan_ = true;
  goal_reached_ = false;
  stand_at_goal_ = false;
  costmap_ros_ = costmap_ros;

  //Parameter for dynamic reconfigure
  dsrv_ = new dynamic_reconfigure::Server<BBPlannerConfig>(private_nh);
  dynamic_reconfigure::Server<BBPlannerConfig>::CallbackType cb = boost::bind(&BBPlanner::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  ROS_INFO("BBPlanner: Version 2 Init.");
}

void BBPlanner::reconfigureCB(BBPlannerConfig &config, uint32_t level) {
  config_ = config;
}

bool BBPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
  global_plan_ = plan;

  //First start of the local plan. First global plan.
  bool first_use = false;
  if (first_setPlan_) {
    first_setPlan_ = false;
    first_use = true;
    bitbots_local_planner::getXPose(*tf_buffer_,
                                    global_plan_,
                                    costmap_ros_->getGlobalFrameID(),
                                    old_goal_pose_,
                                    global_plan_.size() - 1);
    first_use = true;
  }

  bitbots_local_planner::getXPose(*tf_buffer_,
                                  global_plan_,
                                  costmap_ros_->getGlobalFrameID(),
                                  goal_pose_,
                                  global_plan_.size() - 1);

  //Have the new global plan an new goal, reset. Else dont reset.
  if (std::abs(std::abs(old_goal_pose_.getOrigin().getX()) - std::abs(goal_pose_.getOrigin().getX()))
      <= config_.position_accuracy &&
      std::abs(std::abs(old_goal_pose_.getOrigin().getY()) - std::abs(goal_pose_.getOrigin().getY()))
          <= config_.position_accuracy && !first_use
      && std::abs(angles::shortest_angular_distance(tf2::getYaw(old_goal_pose_.getRotation()),
                                                    tf2::getYaw(goal_pose_.getRotation())))
          <= config_.rotation_accuracy) {
    ROS_DEBUG("BBPlanner: Old Goal == new Goal.");
  } else {
    goal_reached_ = false;
    stand_at_goal_ = false;
    ROS_INFO("BBPlanner: New Goal. Start new routine.");
  }

  old_goal_pose_ = goal_pose_;

  return true;
}

bool BBPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  ros::Time begin = ros::Time::now();

  tf2::Stamped<tf2::Transform> current_pose;
  geometry_msgs::PoseStamped msg;
  costmap_ros_->getRobotPose(msg);
  current_pose.setData(
      tf2::Transform(
          tf2::Quaternion(
              msg.pose.orientation.x,
              msg.pose.orientation.y,
              msg.pose.orientation.z,
              msg.pose.orientation.w),
          tf2::Vector3(
              msg.pose.position.x,
              msg.pose.position.y,
              msg.pose.position.z)));

  double walk_angle = std::fmod(
      std::atan2(
          goal_pose_.getOrigin().y() - current_pose.getOrigin().y(),
          goal_pose_.getOrigin().x() - current_pose.getOrigin().x()),
      2 * M_PI);

  double distance = sqrt(
      pow(goal_pose_.getOrigin().y() - current_pose.getOrigin().y(), 2) +
          pow(goal_pose_.getOrigin().x() - current_pose.getOrigin().x(), 2)
  );

  double walk_vel = std::min(distance * config_.translation_slow_down_factor, config_.max_vel_x);

  double diff = 0;
  if (distance > config_.orient_to_goal_distance) {
    diff = walk_angle - tf2::getYaw(current_pose.getRotation());
  } else {
    diff = tf2::getYaw(goal_pose_.getRotation()) - tf2::getYaw(current_pose.getRotation());
  }

  double min_angle = (std::fmod(diff + M_PI, 2 * M_PI) - M_PI);
  double vel = std::max(std::min(
      config_.rotation_slow_down_factor * min_angle,
      config_.max_rotation_vel), -config_.max_rotation_vel);

  if (distance < config_.position_accuracy && abs(min_angle) < config_.rotation_accuracy) {
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    goal_reached_ = true;
  } else {
    cmd_vel.linear.x = std::cos(walk_angle - tf2::getYaw(current_pose.getRotation())) * walk_vel;
    cmd_vel.linear.y = std::sin(walk_angle - tf2::getYaw(current_pose.getRotation())) * walk_vel;
    cmd_vel.angular.z = vel;
    goal_reached_ = false;
  }

  publishPlan(0);

  ros::Time end = ros::Time::now();
  ros::Duration duration = end - begin;
  ROS_DEBUG("BBPlanner: Calculation time: %f seconds", duration.toSec());
  return true;
}

bool BBPlanner::isGoalReached() {
  if (goal_reached_) {
    ROS_INFO("BBPlanner: Goal reached.");
  }
  return goal_reached_;
}

void BBPlanner::publishPlan(int max_point) {
  std::vector<geometry_msgs::PoseStamped> path;
  path = transformed_global_plan_;

  //given an empty path we won't do anything
  if (path.empty())
    return;

  //create a path message
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = path[0].header.frame_id;
  gui_path.header.stamp = path[0].header.stamp;

  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for (unsigned int i = 0; i < path.size(); i++) {
    gui_path.poses[i] = path[i];
  }

  local_plan_publisher_.publish(gui_path);
}

BBPlanner::~BBPlanner() {
}
}
