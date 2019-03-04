#pragma once

#include <Eigen/Geometry>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/MoveGroupActionResult.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>
#include <urdf/model.h>
#include <urdf_model/model.h>


inline double mix(double a, double b, double f) {
  return a * (1.0 - f) + b * f;
}

inline tf2::Vector3 mix(const tf2::Vector3 &a, const tf2::Vector3 &b, double f) {
  return a * (1.0 - f) + b * f;
}

inline Eigen::Vector3d mix(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                           double f) {
  return a * (1.0 - f) + b * f;
}

inline Eigen::Isometry3d mix(const Eigen::Isometry3d &a, const Eigen::Isometry3d &b,
                           double f) {
  Eigen::Isometry3d ret = Eigen::Isometry3d::Identity();
  ret.rotate(Eigen::Quaterniond(a.rotation())
                 .slerp(f, Eigen::Quaterniond(b.rotation())));
  ret.translation() = a.translation() * (1.0 - f) + b.translation() * f;
  return ret;
}

inline tf2::Quaternion mix(const tf2::Quaternion &a, const tf2::Quaternion &b,
                          double f) {
  return a.slerp(b, f);
}


class RobotStatePublisher {
  std::string prefix;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  std::vector<geometry_msgs::TransformStamped> msgs;

public:
  RobotStatePublisher(const std::string &prefix) : prefix(prefix) {}
  void publish(const robot_state::RobotState &robot_state) {
    if (prefix.empty())
      return;
    auto robot_model = robot_state.getRobotModel();
    auto time = ros::Time::now();
    msgs.clear();
    for (auto &link_name : robot_model->getLinkModelNames()) {
      auto f = robot_state.getFrameTransform(link_name);
      Eigen::Quaterniond q(f.rotation());
      msgs.emplace_back();
      auto &msg = msgs.back();
      msg.header.stamp = time;
      msg.header.frame_id = "world";
      msg.child_frame_id = prefix + "/" + link_name;
      msg.transform.translation.x = f.translation().x();
      msg.transform.translation.y = f.translation().y();
      msg.transform.translation.z = f.translation().z();
      msg.transform.rotation.x = q.x();
      msg.transform.rotation.y = q.y();
      msg.transform.rotation.z = q.z();
      msg.transform.rotation.w = q.w();
    }
    tf_broadcaster.sendTransform(msgs);
  }
};
