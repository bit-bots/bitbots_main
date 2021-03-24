#ifndef FTC_LOCAL_PLANNER_TRANSFORM_GLOBAL_PLAN_H_
#define FTC_LOCAL_PLANNER_TRANSFORM_GLOBAL_PLAN_H_
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <string>
#include <cmath>

#include <angles/angles.h>
#include <costmap_2d/costmap_2d.h>

namespace bitbots_local_planner {
/**
* @brief  Returns X pose in plan
* @param tf A reference to a transform buffer
* @param global_plan The plan being followed
* @param global_frame The global frame of the local planner
* @param goal_pose the pose to copy into
* @return True if achieved, false otherwise
*/
bool getXPose(const tf2_ros::Buffer &tf,
              const std::vector<geometry_msgs::PoseStamped> &global_plan,
              const std::string &global_frame,
              tf::Stamped<tf::Pose> &goal_pose, int plan_point);
};
#endif
