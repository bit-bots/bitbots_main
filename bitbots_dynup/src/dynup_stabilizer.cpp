#include "bitbots_dynup/dynup_stabilizer.h"

#include <memory>
#include <utility>
#include <bitbots_splines/dynamic_balancing_goal.h>
#include <bitbots_splines/reference_goals.h>

namespace bitbots_dynup {

void Stabilizer::init(moveit::core::RobotModelPtr kinematic_model) {
  kinematic_model_ = std::move(kinematic_model);

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
  cop_x_error_ = 0;
  cop_y_error_ = 0;
  cop_x_error_sum_ = 0;
  cop_y_error_sum_ = 0;
}

DynupResponse Stabilizer::stabilize(const DynupResponse &ik_goals, const ros::Duration &dt) {
    tf2::Transform to_trunk_tf;
    tf2::fromMsg(to_trunk_.transform, to_trunk_tf);
    tf2::Transform trunk_goal = ik_goals.r_foot_goal_pose * to_trunk_tf;

    if(use_stabilizing_) {
        if(stabilize_now_) {
            double cop_x_error, cop_y_error;
            cop_x_error = cop_.x - trunk_goal.getOrigin().getX();
            cop_y_error = cop_.y - trunk_goal.getOrigin().getY();
            cop_x_error_sum_ += cop_x_error;
            cop_y_error_sum_ += cop_y_error;
            double x = trunk_goal.getOrigin().getX() - cop_.x * p_x_factor_ - i_x_factor_ * cop_x_error_sum_ -
                       d_x_factor_ * (cop_x_error - cop_x_error_);
            double y = trunk_goal.getOrigin().getY() - cop_.y * p_y_factor_ - i_y_factor_ * cop_y_error_sum_ -
                       d_y_factor_ * (cop_y_error - cop_y_error_);
            cop_x_error_ = cop_x_error;
            cop_y_error_ = cop_y_error;
            trunk_goal.setOrigin({x, y, trunk_goal.getOrigin().getZ()});
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

void Stabilizer::setCoP(geometry_msgs::Point cop) {
    cop_ = cop;
}

void Stabilizer::setPFactor(double factor_x, double factor_y) {
    p_x_factor_ = factor_x;
    p_y_factor_ = factor_y;
}

void Stabilizer::setIFactor(double factor_x, double factor_y) {
    i_x_factor_ = factor_x;
    i_y_factor_ = factor_y;
}

void Stabilizer::setDFactor(double factor_x, double factor_y) {
    d_x_factor_ = factor_x;
    d_y_factor_ = factor_y;
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
