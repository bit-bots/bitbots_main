#include "bitbots_dynamic_kick/kick_engine.h"

#include <utility>

namespace bitbots_dynamic_kick {

KickEngine::KickEngine() {
}

void KickEngine::reset() {
  time_ = 0;
  trunk_spline_ = bitbots_splines::PoseSpline();
  flying_foot_spline_ = bitbots_splines::PoseSpline();
}

void KickEngine::setGoals(const KickGoals &goals) {
  is_left_kick_ = calcIsLeftFootKicking(goals.ball_position, goals.kick_direction);
  // TODO Internal state is dirty when goal transformation fails

  /* Save given goals because we reuse them later */
  auto transformed_goal = transformGoal((is_left_kick_) ? "r_sole" : "l_sole",
                                        goals.trunk_to_base_footprint, goals.ball_position, goals.kick_direction);
  ball_position_ = transformed_goal.first;
  kick_direction_ = transformed_goal.second;
  kick_direction_.normalize();
  kick_speed_ = goals.kick_speed;

  time_ = 0;

  Eigen::Isometry3d trunk_to_flying_foot = current_state_->getGlobalLinkTransform(is_left_kick_ ? "l_sole" : "r_sole");
  Eigen::Isometry3d trunk_to_support_foot = current_state_->getGlobalLinkTransform(is_left_kick_ ? "r_sole" : "l_sole");

  /* Plan new splines according to new goal */
  calcSplines(trunk_to_support_foot.inverse() * trunk_to_flying_foot, getTrunkPose());
}

KickPositions KickEngine::update(double dt) {
  /* Only do an actual update when splines are present */
  KickPositions positions;
  /* Get should-be pose from planned splines (every axis) at current time */
  positions.trunk_pose = tf2::transformToEigen(tf2::toMsg(trunk_spline_.getTfTransform(time_)));
  positions.flying_foot_pose = tf2::transformToEigen(tf2::toMsg(flying_foot_spline_.getTfTransform(time_)));
  positions.is_left_kick = is_left_kick_;
  positions.engine_time = time_;

  /* calculate if we want to use center-of-pressure in the current phase
   * use COP based support point only when the weight is on the support foot
   * while raising/lowering the foot, the weight is not completely on the support foot (that's why /2.0)*/
  if (time_ > params_.move_trunk_time + params_.raise_foot_time / 2.0 &&
      time_ < phase_timings_.move_back + params_.lower_foot_time / 2.0) {
    positions.cop_support_point = true;
  } else {
    positions.cop_support_point = false;
  }

  time_ += dt;

  /* Stabilize and return result */
  return positions;
}

Eigen::Isometry3d KickEngine::getTrunkPose() {
  Eigen::Isometry3d trunk_frame;
  if (is_left_kick_) {
    trunk_frame = current_state_->getGlobalLinkTransform("r_sole").inverse();
  } else {
    trunk_frame = current_state_->getGlobalLinkTransform("l_sole").inverse();
  }
  return trunk_frame;
}

void KickEngine::calcSplines(const Eigen::Isometry3d &flying_foot_pose, const Eigen::Isometry3d &trunk_pose) {
  /*
   * Add current position, target position and current position to splines so that they describe a smooth
   * curve to the ball and back
   */
  /* Splines:
   * - stand
   * - move trunk
   * - raise foot
   * - kick
   * - move foot back
   * - lower foot
   * - move trunk back
   */

  /* calculate timings for this kick */
  phase_timings_.move_trunk = 0 + params_.move_trunk_time;
  phase_timings_.raise_foot = phase_timings_.move_trunk + params_.raise_foot_time;
  phase_timings_.windup = phase_timings_.raise_foot + params_.move_to_ball_time;
  phase_timings_.kick = phase_timings_.windup + params_.kick_time;
  phase_timings_.move_back = phase_timings_.kick + params_.move_back_time;
  phase_timings_.lower_foot = phase_timings_.move_back + params_.lower_foot_time;
  phase_timings_.move_trunk_back = phase_timings_.lower_foot + params_.move_trunk_back_time;

  int kick_foot_sign;
  if (is_left_kick_) {
    kick_foot_sign = 1;
  } else {
    kick_foot_sign = -1;
  }

  windup_point_ = calcKickWindupPoint();

  /* build vector of speeds in each direction */
  double speed_yaw = rot_conv::EYawOfQuat(kick_direction_);
  Eigen::Vector3d speed_vector(cos(speed_yaw), sin(speed_yaw), 0);

  /* Flying foot position */
  flying_foot_spline_.x()->addPoint(0, flying_foot_pose.translation().x());
  flying_foot_spline_.x()->addPoint(phase_timings_.move_trunk, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.raise_foot, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.windup, windup_point_.x(), 0, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.kick, ball_position_.x(),
                                    speed_vector.x() * kick_speed_, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.move_back, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.lower_foot, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.move_trunk_back, 0);

  flying_foot_spline_.y()->addPoint(0, flying_foot_pose.translation().y());
  flying_foot_spline_.y()->addPoint(phase_timings_.move_trunk, kick_foot_sign * params_.foot_distance);
  flying_foot_spline_.y()->addPoint(phase_timings_.raise_foot, kick_foot_sign * params_.foot_distance);
  flying_foot_spline_.y()->addPoint(phase_timings_.windup, windup_point_.y(), 0, 0);
  flying_foot_spline_.y()
      ->addPoint(phase_timings_.kick, ball_position_.y(), speed_vector.y() * kick_speed_, 0);
  flying_foot_spline_.y()->addPoint(phase_timings_.move_back, kick_foot_sign * params_.foot_distance);
  flying_foot_spline_.y()->addPoint(phase_timings_.lower_foot, kick_foot_sign * params_.foot_distance);
  flying_foot_spline_.y()->addPoint(phase_timings_.move_trunk_back, kick_foot_sign * params_.foot_distance);

  flying_foot_spline_.z()->addPoint(0, flying_foot_pose.translation().z());
  flying_foot_spline_.z()->addPoint(phase_timings_.move_trunk, 0);
  flying_foot_spline_.z()->addPoint(phase_timings_.raise_foot, params_.foot_rise);
  flying_foot_spline_.z()->addPoint(phase_timings_.windup, params_.foot_rise);
  flying_foot_spline_.z()->addPoint(phase_timings_.kick, params_.foot_rise);
  flying_foot_spline_.z()->addPoint(phase_timings_.move_back, params_.foot_rise);
  flying_foot_spline_.z()->addPoint(phase_timings_.lower_foot, 0.4 * params_.foot_rise);
  flying_foot_spline_.z()->addPoint(phase_timings_.move_trunk_back, 0);

  /* Flying foot orientation */
  /* Get euler angles for foot rotation */
  double start_r, start_p, start_y;
  Eigen::Quaterniond flying_foot_rotation(flying_foot_pose.rotation());
  rot_conv::EulerFromQuat(flying_foot_rotation, start_y, start_p, start_r);
  double target_y = calcKickFootYaw();

  /* Add these quaternions in the same fashion as before to our splines (current, target, current) */
  flying_foot_spline_.roll()->addPoint(0, start_r);
  flying_foot_spline_.roll()->addPoint(phase_timings_.windup, start_r);
  flying_foot_spline_.roll()->addPoint(phase_timings_.move_trunk_back, start_r);
  flying_foot_spline_.pitch()->addPoint(0, start_p);
  flying_foot_spline_.pitch()->addPoint(phase_timings_.windup, start_p);
  flying_foot_spline_.pitch()->addPoint(phase_timings_.move_trunk_back, start_p);
  flying_foot_spline_.yaw()->addPoint(0, start_y);
  flying_foot_spline_.yaw()->addPoint(phase_timings_.raise_foot, start_y);
  flying_foot_spline_.yaw()->addPoint(phase_timings_.windup, target_y);
  flying_foot_spline_.yaw()->addPoint(phase_timings_.kick, target_y);
  flying_foot_spline_.yaw()->addPoint(phase_timings_.move_back, start_y);
  flying_foot_spline_.yaw()->addPoint(phase_timings_.move_trunk_back, start_y);

  /* Stabilizing point */
  trunk_spline_.x()->addPoint(0, 0);
  trunk_spline_.x()->addPoint(phase_timings_.move_trunk, params_.stabilizing_point_x);
  trunk_spline_.x()->addPoint(phase_timings_.raise_foot, params_.stabilizing_point_x);
  trunk_spline_.x()->addPoint(phase_timings_.windup, params_.stabilizing_point_x);
  trunk_spline_.x()->addPoint(phase_timings_.kick, params_.stabilizing_point_x);
  trunk_spline_.x()->addPoint(phase_timings_.move_back, params_.stabilizing_point_x);
  trunk_spline_.x()->addPoint(phase_timings_.lower_foot, params_.stabilizing_point_x);
  trunk_spline_.x()->addPoint(phase_timings_.move_trunk_back, 0);

  trunk_spline_.y()->addPoint(0, kick_foot_sign * (params_.foot_distance / 2.0));
  trunk_spline_.y()
      ->addPoint(phase_timings_.move_trunk, kick_foot_sign * (-params_.stabilizing_point_y));
  trunk_spline_.y()
      ->addPoint(phase_timings_.raise_foot, kick_foot_sign * (-params_.stabilizing_point_y));
  trunk_spline_.y()
      ->addPoint(phase_timings_.windup, kick_foot_sign * (-params_.stabilizing_point_y));
  trunk_spline_.y()
      ->addPoint(phase_timings_.kick, kick_foot_sign * (-params_.stabilizing_point_y));
  trunk_spline_.y()
      ->addPoint(phase_timings_.move_back, kick_foot_sign * (-params_.stabilizing_point_y));
  trunk_spline_.y()
      ->addPoint(phase_timings_.lower_foot, kick_foot_sign * (-params_.stabilizing_point_y));
  trunk_spline_.y()
      ->addPoint(phase_timings_.move_trunk_back, kick_foot_sign * (params_.foot_distance / 2.0));

  trunk_spline_.z()->addPoint(0, trunk_pose.translation().z());
  trunk_spline_.z()->addPoint(phase_timings_.move_trunk, params_.trunk_height);
  trunk_spline_.z()->addPoint(phase_timings_.lower_foot, params_.trunk_height);
  trunk_spline_.z()->addPoint(phase_timings_.move_trunk_back, trunk_pose.translation().z());

  /* Get euler angles for trunk rotation */
  double trunk_r, trunk_p, trunk_y;
  Eigen::Quaterniond trunk_rotation(trunk_pose.rotation());
  rot_conv::EulerFromQuat(flying_foot_rotation, trunk_y, trunk_p, trunk_r);

  trunk_spline_.roll()->addPoint(0, trunk_r);
  trunk_spline_.roll()->addPoint(phase_timings_.raise_foot, kick_foot_sign * params_.trunk_roll);
  trunk_spline_.roll()->addPoint(phase_timings_.lower_foot, kick_foot_sign * params_.trunk_roll);
  trunk_spline_.roll()->addPoint(phase_timings_.move_trunk_back, trunk_r);
  trunk_spline_.pitch()->addPoint(0, trunk_p);
  trunk_spline_.pitch()->addPoint(phase_timings_.raise_foot, params_.trunk_pitch);
  trunk_spline_.pitch()->addPoint(phase_timings_.lower_foot, params_.trunk_pitch);
  trunk_spline_.pitch()->addPoint(phase_timings_.move_trunk_back, trunk_p);
  trunk_spline_.yaw()->addPoint(0, trunk_y);
  trunk_spline_.yaw()->addPoint(phase_timings_.raise_foot, kick_foot_sign * params_.trunk_yaw);
  trunk_spline_.yaw()->addPoint(phase_timings_.lower_foot, kick_foot_sign * params_.trunk_yaw);
  trunk_spline_.yaw()->addPoint(phase_timings_.move_trunk_back, trunk_y);
}

std::pair<Eigen::Vector3d, Eigen::Quaterniond> KickEngine::transformGoal(
    const std::string &support_foot_frame,
    const Eigen::Isometry3d &trunk_to_base_footprint,
    const Eigen::Vector3d &ball_position,
    const Eigen::Quaterniond &kick_direction) {
  /* ball_position and kick_direction are currently in base_footprint, we want them in support foot frame */
  /* first, get transform from base_footprint to support foot */
  Eigen::Isometry3d trunk_to_support_foot = current_state_->getGlobalLinkTransform(support_foot_frame);
  Eigen::Isometry3d base_footprint_to_support_foot = trunk_to_base_footprint.inverse() * trunk_to_support_foot;
  /* now, apply the transforms. Because of eigen, the transform has to be on the left hand side, therefore it must be inversed */
  Eigen::Vector3d ball_transformed = base_footprint_to_support_foot.inverse() * ball_position;
  Eigen::Matrix3d kick_direction_transformed_matrix = (base_footprint_to_support_foot.inverse() * kick_direction).rotation();
  Eigen::Quaterniond kick_direction_transformed(kick_direction_transformed_matrix);
  return std::make_pair(ball_transformed, kick_direction_transformed);
}

Eigen::Vector3d KickEngine::calcKickWindupPoint() {
  /* retrieve yaw from kick_direction_ */
  double yaw = rot_conv::EYawOfQuat(kick_direction_);

  /* create a vector which points in the negative direction of kick_direction_ */
  Eigen::Vector3d vec(cos(yaw), sin(yaw), 0);
  vec.normalize();

  /* take windup distance into account */
  vec *= -params_.kick_windup_distance;

  /* add the ball position because the windup point is in support_foot_frame and not ball_frame */
  vec += ball_position_;

  vec.z() = params_.foot_rise;

  return vec;
}

bool KickEngine::calcIsLeftFootKicking(const Eigen::Vector3d &ball_position,
                                       const Eigen::Quaterniond &kick_direction) {
  /* it is important that everything is in base_footprint frame! */

  /*
   * check if ball is outside of an imaginary corridor
   * if it is not, we use a more fined grained criterion which takes kick_direction into account
   */
  if (ball_position.y() > params_.choose_foot_corridor_width / 2) {
    return true;
  } else if (ball_position.y() < -params_.choose_foot_corridor_width / 2) {
    return false;
  }

  /* use the more fine grained angle based criterion
   * angle_1 = angle between "forward" and "ball position"
   * angle_2 = yaw of kick_direction
   * angle_diff = difference between the two based on which the decision happens
   */
  double angle_1 = std::atan2(ball_position.y(), ball_position.x());

  double angle_2 = rot_conv::EYawOfQuat(kick_direction);
  double angle_diff = angle_2 - angle_1;

  ROS_INFO_STREAM("Choosing " << ((angle_diff < 0) ? "left" : "right") << " foot to kick");

  return angle_diff < 0;
}

double KickEngine::calcKickFootYaw() {
  double kick_yaw_angle = rot_conv::EYawOfQuat(kick_direction_);

  if (kick_yaw_angle > M_PI_4) {
    return kick_yaw_angle - M_PI_2;
  } else if (kick_yaw_angle < -M_PI_4) {
    return kick_yaw_angle + M_PI_2;
  } else {
    return kick_yaw_angle;
  }
}

bool KickEngine::isLeftKick() {
  return is_left_kick_;
}

int KickEngine::getPercentDone() const {
  return int(time_ / phase_timings_.move_trunk_back * 100);
}

bitbots_splines::PoseSpline KickEngine::getFlyingSplines() const {
  return flying_foot_spline_;
}

bitbots_splines::PoseSpline KickEngine::getTrunkSplines() const {
  return trunk_spline_;
}

KickPhase KickEngine::getPhase() const {
  if (time_ == 0)
    return KickPhase::INITIAL;
  else if (time_ <= phase_timings_.move_trunk)
    return KickPhase::MOVE_TRUNK;
  else if (time_ <= phase_timings_.raise_foot)
    return KickPhase::RAISE_FOOT;
  else if (time_ <= phase_timings_.windup)
    return KickPhase::WINDUP;
  else if (time_ <= phase_timings_.kick)
    return KickPhase::KICK;
  else if (time_ <= phase_timings_.move_back)
    return KickPhase::MOVE_BACK;
  else if (time_ <= phase_timings_.lower_foot)
    return KickPhase::LOWER_FOOT;
  else if (time_ <= phase_timings_.move_trunk_back)
    return KickPhase::MOVE_TRUNK_BACK;
  else
    return KickPhase::DONE;
}

double KickEngine::getTime() const {
  return time_;
}

void KickEngine::setParams(KickParams params) {
  params_ = params;
}

Eigen::Vector3d KickEngine::getWindupPoint() {
  return windup_point_;
}

void KickEngine::setRobotState(robot_state::RobotStatePtr current_state) {
  current_state_ = current_state;
}

}
