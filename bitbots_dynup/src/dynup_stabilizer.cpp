#include "bitbots_dynup/dynup_stabilizer.h"

#include <memory>
#include <utility>
#include <bitbots_splines/dynamic_balancing_goal.h>
#include <bitbots_splines/reference_goals.h>


namespace bitbots_dynup {

void Stabilizer::init(moveit::core::RobotModelPtr kinematic_model) {
  kinematic_model_ = std::move(kinematic_model);
  ros::NodeHandle nh = ros::NodeHandle("/dynup/pid");
  pid_trunk_pitch_.init(nh, false);
  pid_trunk_pitch_.reset();
  pid_trunk_roll_.init(nh, false);
  pid_trunk_roll_.reset();

  /* Reset kinematic goal to default */
  goal_state_.reset(new robot_state::RobotState(kinematic_model_));
  goal_state_->setToDefaultValues();
}

void Stabilizer::reset() {
  /* We have to set some good initial position in the goal state,
   * since we are using a gradient based method. Otherwise, the
   * first step will be not correct */
  std::vector<std::string> names_vec =
      {"HeadPan", "HeadTilt", "LAnklePitch", "LAnkleRoll", "LElbow", "LHipPitch", "LHipRoll", "LHipYaw", "LKnee",
       "LShoulderPitch", "LShoulderRoll", "RAnklePitch", "RAnkleRoll", "RElbow", "RHipPitch", "RHipRoll", "RHipYaw",
       "RKnee", "RShoulderPitch", "RShoulderRoll"};
  std::vector<double> pos_vec = {0, 0, -25, 4, 45, 27, 4, -1, -58, 0, 25, -4, 45, -37, -4, 1, 0, 58, 0, 0};
  for (double &pos: pos_vec) {
    pos = pos / 180.0 * M_PI;
  }
  for (int i = 0; i < names_vec.size(); i++) {
    goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
  }
}

DynupResponse Stabilizer::stabilize(const DynupResponse &ik_goals, const ros::Duration &dt) {
    tf2::Transform to_trunk_tf;
    tf2::fromMsg(to_trunk_.transform, to_trunk_tf);
    tf2::Transform trunk_goal = ik_goals.r_foot_goal_pose * to_trunk_tf;

    if(use_stabilizing_) {
        if(stabilize_now_) {
            double goal_pitch, goal_roll, goal_yaw;
            tf2::Matrix3x3(trunk_goal.getRotation()).getRPY(goal_roll, goal_pitch, goal_yaw);
            // first adapt trunk pitch value based on PID controller
            double corrected_pitch = pid_trunk_pitch_.computeCommand(goal_pitch - cop_.x, dt);
            double corrected_roll = pid_trunk_roll_.computeCommand(goal_roll - cop_.y, dt);
            tf2::Quaternion corrected_orientation;
            corrected_orientation.setRPY(goal_roll + corrected_roll, goal_pitch + corrected_pitch, goal_yaw);

            trunk_goal.setRotation(corrected_orientation);
        }
    }

    tf2::Transform from_trunk_tf;
    tf2::fromMsg(from_trunk_.transform, from_trunk_tf);
    tf2::Transform right_foot_goal = trunk_goal * from_trunk_tf;
    tf2::Transform left_foot_goal = ik_goals.l_foot_goal_pose * ik_goals.r_foot_goal_pose;
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

void Stabilizer::setStabilizeNow(bool now) {
    stabilize_now_ = now;
}

void Stabilizer::useMinimalDisplacement(bool use) {
  use_minimal_displacement_ = use;
}

void Stabilizer::setStabilizingWeight(double weight) {
  stabilizing_weight_ = weight;
}

void Stabilizer::setRobotModel(moveit::core::RobotModelPtr model) {
  kinematic_model_ = std::move(model);
}

void Stabilizer::setTransforms(geometry_msgs::TransformStamped to_trunk, geometry_msgs::TransformStamped from_trunk) {
    to_trunk_ = to_trunk;
    from_trunk_ = from_trunk;
}

}
