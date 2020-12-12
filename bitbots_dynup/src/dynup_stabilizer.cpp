#include "bitbots_dynup/dynup_stabilizer.h"

#include <memory>
#include <utility>
#include <bitbots_splines/dynamic_balancing_goal.h>


namespace bitbots_dynup {

void Stabilizer::init(moveit::core::RobotModelPtr kinematic_model) {
  kinematic_model_ = std::move(kinematic_model);
  ros::NodeHandle nhp = ros::NodeHandle("dynup/pid_trunk_pitch");
  ros::NodeHandle nhr = ros::NodeHandle("dynup/pid_trunk_roll");

  pid_trunk_pitch_.init(nhp, false);
  pid_trunk_roll_.init(nhr, false);

  /* Reset kinematic goal to default */
  goal_state_.reset(new robot_state::RobotState(kinematic_model_));
  goal_state_->setToDefaultValues();
}

void Stabilizer::reset() {
  pid_trunk_pitch_.reset();
  pid_trunk_roll_.reset();

}

DynupResponse Stabilizer::stabilize(const DynupResponse &ik_goals, const ros::Duration &dt) {
    tf2::Transform right_foot_goal;
    tf2::Quaternion quat;
    tf2::convert(imu_.orientation, quat);
    // the tf2::Quaternion has a method to access roll pitch and yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if(use_stabilizing_ && ik_goals.is_stabilizing_needed) {
        tf2::Transform to_trunk_tf;
        tf2::fromMsg(to_trunk_.transform, to_trunk_tf);
        tf2::Transform trunk_goal = ik_goals.r_foot_goal_pose * to_trunk_tf;

        double goal_pitch, goal_roll, goal_yaw;
        tf2::Matrix3x3(trunk_goal.getRotation()).getRPY(goal_roll, goal_pitch, goal_yaw);
        // first adapt trunk pitch value based on PID controller
        double corrected_pitch = pid_trunk_pitch_.computeCommand(goal_pitch - pitch, dt);
        double corrected_roll = pid_trunk_roll_.computeCommand(goal_roll - roll, dt);
        tf2::Quaternion corrected_orientation;
        corrected_orientation.setRPY(goal_roll + corrected_roll, goal_pitch + corrected_pitch, goal_yaw);
        trunk_goal.setRotation(corrected_orientation);

        right_foot_goal = trunk_goal * to_trunk_tf.inverse() ;
    }
    else {
        right_foot_goal = ik_goals.r_foot_goal_pose;
    }

    tf2::Transform left_foot_goal = ik_goals.l_foot_goal_pose * right_foot_goal;
    tf2::Transform left_hand_goal = ik_goals.l_hand_goal_pose;
    tf2::Transform right_hand_goal = ik_goals.r_hand_goal_pose;

    DynupResponse response;
    response.r_foot_goal_pose = right_foot_goal;
    response.l_foot_goal_pose = left_foot_goal;
    response.r_hand_goal_pose = right_hand_goal;
    response.l_hand_goal_pose = left_hand_goal;

    return response;
}

void Stabilizer::useStabilizing(bool use) {
  use_stabilizing_ = use;
}


void Stabilizer::setRobotModel(moveit::core::RobotModelPtr model) {
  kinematic_model_ = std::move(model);
}

void Stabilizer::setTransforms(geometry_msgs::TransformStamped to_trunk) {
    to_trunk_ = to_trunk;
}

}
