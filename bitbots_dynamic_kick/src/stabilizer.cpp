#include <utility>

#include "bitbots_dynamic_kick/stabilizer.h"
#include "bitbots_splines/dynamic_balancing_goal.h"
#include "bitbots_splines/reference_goals.h"

namespace bitbots_dynamic_kick {

Stabilizer::Stabilizer() {
  reset();
}

void Stabilizer::reset() {
  cop_x_error_sum_ = 0.0;
  cop_y_error_sum_ = 0.0;
}

std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> Stabilizer::stabilize(const KickPositions &positions) {
  /* ik options is basically the command which we send to bio_ik and which describes what we want to do */
  auto ik_options = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
  ik_options->replace = true;
  ik_options->return_approximate_solution = true;

  tf2::Transform trunk_goal;
  if (positions.cop_support_point && use_cop_) {
    /* calculate stabilizing target from center of pressure
     * the cop is in corresponding sole frame
     * optimal stabilizing would be centered above sole center */
    double cop_x, cop_y, cop_x_error, cop_y_error;
    if (positions.is_left_kick) {
      cop_x = cop_right.x;
      cop_y = cop_right.y;
    } else {
      cop_x = cop_left.x;
      cop_y = cop_left.y;
    }
    cop_x_error = cop_x - positions.trunk_pose.position.x;
    cop_y_error = cop_y - positions.trunk_pose.position.y;
    cop_x_error_sum_ += cop_x_error;
    cop_y_error_sum_ += cop_y_error;
    double x = positions.trunk_pose.position.x - cop_x*p_x_factor_ - i_x_factor_*cop_x_error_sum_
                                - d_x_factor_*(cop_x_error - cop_x_error_);
    double y = positions.trunk_pose.position.y - cop_y*p_y_factor_ - i_y_factor_*cop_y_error_sum_
                                - d_y_factor_*(cop_y_error - cop_y_error_);
    cop_x_error_ = cop_x_error;
    cop_y_error_ = cop_y_error;
    /* Do not use control for height and rotation */
    trunk_goal.setOrigin({x, y, positions.trunk_pose.position.z});
    trunk_goal.setRotation({positions.trunk_pose.orientation.x, positions.trunk_pose.orientation.y,
                            positions.trunk_pose.orientation.z, positions.trunk_pose.orientation.w});
  } else {
    tf2::convert(positions.trunk_pose, trunk_goal);
  }
  auto *bio_ik_trunk_goal = new ReferencePoseGoal();
  bio_ik_trunk_goal->setWeight(trunk_weight_);
  bio_ik_trunk_goal->setPosition(trunk_goal.getOrigin());
  bio_ik_trunk_goal->setOrientation(trunk_goal.getRotation());
  bio_ik_trunk_goal->setLinkName("base_link");
  if (positions.is_left_kick) {
    bio_ik_trunk_goal->setReferenceLinkName("r_sole");
  } else {
    bio_ik_trunk_goal->setReferenceLinkName("l_sole");
  }
  ik_options->goals.emplace_back(bio_ik_trunk_goal);

  tf2::Transform flying_foot_goal;
  flying_foot_goal.setOrigin({positions.flying_foot_pose.position.x,
                              positions.flying_foot_pose.position.y,
                              positions.flying_foot_pose.position.z});
  flying_foot_goal.setRotation({positions.flying_foot_pose.orientation.x,
                                positions.flying_foot_pose.orientation.y,
                                positions.flying_foot_pose.orientation.z,
                                positions.flying_foot_pose.orientation.w});


  /* construct the bio_ik Pose object which tells bio_ik what we want to achieve */
  auto *bio_ik_flying_foot_goal = new ReferencePoseGoal();
  bio_ik_flying_foot_goal->setPosition(flying_foot_goal.getOrigin());
  bio_ik_flying_foot_goal->setOrientation(flying_foot_goal.getRotation());
  if (positions.is_left_kick) {
    bio_ik_flying_foot_goal->setLinkName("l_sole");
    bio_ik_flying_foot_goal->setReferenceLinkName("r_sole");
  } else {
    bio_ik_flying_foot_goal->setLinkName("r_sole");
    bio_ik_flying_foot_goal->setReferenceLinkName("l_sole");
  }
  bio_ik_flying_foot_goal->setWeight(flying_weight_);

  ik_options->goals.emplace_back(bio_ik_flying_foot_goal);
  return std::move(ik_options);
}

void Stabilizer::useCop(bool use) {
  use_cop_ = use;
}

void Stabilizer::setFlyingWeight(double weight) {
  flying_weight_ = weight;
}

void Stabilizer::setTrunkWeight(double weight) {
  trunk_weight_ = weight;
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

void Stabilizer::setRobotModel(moveit::core::RobotModelPtr model) {
  kinematic_model_ = std::move(model);
}

}
