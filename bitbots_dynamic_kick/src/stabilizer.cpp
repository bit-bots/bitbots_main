#include <utility>

#include "bitbots_dynamic_kick/stabilizer.h"
#include "bitbots_dynamic_kick/dynamic_balancing_goal.h"
#include "bitbots_dynamic_kick/reference_goals.h"

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

  tf2::Vector3 stabilizing_target;
  if (positions.cop_support_point && use_cop_) {
    /* calculate stabilizing target from center of pressure
     * the cop is in corresponding sole frame
     * optimal stabilizing would be centered above sole center */
    double cop_x, cop_y, cop_x_error, cop_y_error;
    if (positions.is_left_kick) {
      cop_x = m_cop_right.x;
      cop_y = m_cop_right.y;
    } else {
      cop_x = m_cop_left.x;
      cop_y = m_cop_left.y;
    }
    cop_x_error = cop_x - positions.support_point.x;
    cop_y_error = cop_y - positions.support_point.y;
    cop_x_error_sum_ += cop_x_error;
    cop_y_error_sum_ += cop_y_error;
    stabilizing_target.setX(positions.support_point.x - cop_x*p_x_factor_ - i_x_factor_*cop_x_error_sum_
                                - d_x_factor_*(cop_x_error - cop_x_error_));
    stabilizing_target.setY(positions.support_point.y - cop_y*p_y_factor_ - i_y_factor_*cop_y_error_sum_
                                - d_y_factor_*(cop_y_error - cop_y_error_));
    cop_x_error_ = cop_x_error;
    cop_y_error_ = cop_y_error;
    stabilizing_target.setZ(0);
  } else {
    stabilizing_target = {positions.support_point.x, positions.support_point.y, positions.support_point.z};
  }
  stabilizing_target_ = stabilizing_target;

  tf2::Transform flying_foot_goal;
  flying_foot_goal.setOrigin({positions.flying_foot_pose.pose.position.x,
                              positions.flying_foot_pose.pose.position.y,
                              positions.flying_foot_pose.pose.position.z});
  flying_foot_goal.setRotation({positions.flying_foot_pose.pose.orientation.x,
                                positions.flying_foot_pose.pose.orientation.y,
                                positions.flying_foot_pose.pose.orientation.z,
                                positions.flying_foot_pose.pose.orientation.w});


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

  auto *trunk_orientation_goal = new ReferenceOrientationGoal();
  tf2::Quaternion trunk_orientation;
  trunk_orientation.setRPY(0, 0.2, 0);
  trunk_orientation_goal->setOrientation(trunk_orientation);
  trunk_orientation_goal->setLinkName("base_link");
  if (positions.is_left_kick) {
    trunk_orientation_goal->setReferenceLinkName("r_sole");
  } else {
    trunk_orientation_goal->setReferenceLinkName("l_sole");
  }
  trunk_orientation_goal->setWeight(trunk_orientation_weight_);
  ik_options->goals.emplace_back(trunk_orientation_goal);

  auto *trunk_height_goal = new ReferenceHeightGoal();
  trunk_height_goal->setHeight(trunk_height_);
  trunk_height_goal->setWeight(trunk_height_weight_);
  trunk_height_goal->setLinkName("base_link");
  if (positions.is_left_kick) {
    trunk_height_goal->setReferenceLinkName("r_sole");
  } else {
    trunk_height_goal->setReferenceLinkName("l_sole");
  }
  ik_options->goals.emplace_back(trunk_height_goal);

  auto *bio_ik_balancing_context = new DynamicBalancingContext(kinematic_model_);
  auto *bio_ik_balance_goal =
      new DynamicBalancingGoal(bio_ik_balancing_context, stabilizing_target, stabilizing_weight_);
  if (positions.is_left_kick) {
    bio_ik_balance_goal->setReferenceLink("r_sole");
  } else {
    bio_ik_balance_goal->setReferenceLink("l_sole");
  }

  ik_options->goals.emplace_back(bio_ik_flying_foot_goal);
  if (use_stabilizing_) {
    ik_options->goals.emplace_back(bio_ik_balance_goal);
  }
  if (use_minimal_displacement_) {
    ik_options->goals.emplace_back(new bio_ik::MinimalDisplacementGoal());
  }
  return std::move(ik_options);
}

void Stabilizer::useStabilizing(bool use) {
  use_stabilizing_ = use;
}

void Stabilizer::useMinimalDisplacement(bool use) {
  use_minimal_displacement_ = use;
}

void Stabilizer::useCop(bool use) {
  use_cop_ = use;
}

void Stabilizer::setTrunkHeight(double height) {
  trunk_height_ = height;
}

void Stabilizer::setStabilizingWeight(double weight) {
  stabilizing_weight_ = weight;
}

void Stabilizer::setFlyingWeight(double weight) {
  flying_weight_ = weight;
}

void Stabilizer::setTrunkOrientationWeight(double weight) {
  trunk_orientation_weight_ = weight;
}

void Stabilizer::setTrunkHeightWeight(double weight) {
  trunk_height_weight_ = weight;
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

tf2::Vector3 Stabilizer::getStabilizingTarget() const {
  return stabilizing_target_;
}

void Stabilizer::setRobotModel(moveit::core::RobotModelPtr model) {
  kinematic_model_ = std::move(model);
}

}
