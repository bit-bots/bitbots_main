#include <bitbots_local_planner/transform_global_plan.h>

namespace bitbots_local_planner {

bool getXPose(const tf2_ros::Buffer &tf_buffer,
              const std::vector<geometry_msgs::PoseStamped> &global_plan,
              const std::string &global_frame, tf2::Stamped<tf2::Transform> &goal_pose, int plan_point) {
  if (global_plan.empty()) {
    ROS_ERROR("Received plan with zero length");
    return false;
  }
  if (plan_point >= (int) global_plan.size()) {
    ROS_ERROR("Goal_functions: Plan_point %d to big. Plan size: %lu", plan_point, global_plan.size());
    return false;
  }

  const geometry_msgs::PoseStamped &plan_goal_pose = global_plan.at(plan_point);
  try {
    tf_buffer.canTransform(global_frame, ros::Time::now(),
                           plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                           plan_goal_pose.header.frame_id, ros::Duration(0.5));

    geometry_msgs::TransformStamped tmp = tf_buffer.lookupTransform(global_frame,
                                                                    ros::Time(),
                                                                    plan_goal_pose.header.frame_id,
                                                                    plan_goal_pose.header.stamp,
                                                                    plan_goal_pose.header.frame_id,
                                                                    ros::Duration(0.5));
    tf2::Stamped<tf2::Transform> transform;
    tf2::convert(tmp, transform);

    tf2::convert(plan_goal_pose, goal_pose);
    goal_pose.setData(transform * goal_pose);
    goal_pose.stamp_ = transform.stamp_;
    goal_pose.frame_id_ = global_frame;
  }
  catch (tf2::LookupException &ex) {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch (tf2::ConnectivityException &ex) {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch (tf2::ExtrapolationException &ex) {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (!global_plan.empty()) {
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n",
                global_frame.c_str(),
                (unsigned int) global_plan.size(),
                global_plan[0].header.frame_id.c_str());
    }

    return false;
  }
  return true;
}
}
