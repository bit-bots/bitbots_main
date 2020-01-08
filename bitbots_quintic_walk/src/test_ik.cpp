// from https://github.com/ros-planning/moveit_tutorials/blob/master/doc/robot_model_and_robot_state/src/robot_model_and_robot_state_tutorial.cpp

#include "bitbots_quintic_walk/walk_utils.h"
#include <ros/ros.h>

#include <moveit_msgs/RobotState.h>
#include <humanoid_league_msgs/RobotControlState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/move_group_interface/move_group_interface.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "ik_test");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("/robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("RightLeg");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  ROS_INFO("Set random start position");
  std::vector<double> joint_values;
  kinematic_state->setToRandomPositions(joint_model_group);
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }


  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("r_sole");

  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  ROS_INFO("IK solution");
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  /*
  geometry_msgs::Pose goal_msg;
  goal_msg.position.x = 0.025;
  goal_msg.position.y = -0.070;
  goal_msg.position.z = -0.421;
  goal_msg.orientation.x = 0.0;
  goal_msg.orientation.y = 0.019;
  goal_msg.orientation.z = 0.0;
  goal_msg.orientation.w = 1.0;
  ROS_WARN("IK call");
  bool success = goal_state_->setFromIK(right_leg_joints_group_, goal_msg, 1.0);
  ROS_WARN("success1 %d", success);*/
}
