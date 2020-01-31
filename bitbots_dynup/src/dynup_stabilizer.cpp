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

  //TODO: This is not used!
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

DynupResponse Stabilizer::stabilize(const DynupResponse &response, const ros::Duration &dt) {
  /* ik options is basicaly the command which we send to bio_ik and which describes what we want to do */
  return response;
}

void Stabilizer::useStabilizing(bool use) {
  use_stabilizing_ = use;
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

}
