#include "bitbots_dynamic_kick/kick_engine.h"

namespace bitbots_dynamic_kick {

KickEngine::KickEngine() :
    listener_(tf_buffer_) {
}

void KickEngine::reset() {
  time_ = 0;
  support_point_spline_ = bitbots_splines::PositionSpline();
  flying_foot_spline_ = bitbots_splines::PoseSpline();
}

void KickEngine::setGoals(const KickGoals &goals) {
  is_left_kick_ = calcIsLeftFootKicking(goals.header,
                                        goals.ball_position,
                                        goals.kick_direction);
  // TODO Internal state is dirty when goal transformation fails

  /* Save given goals because we reuse them later */
  auto transformed_goal = transformGoal((is_left_kick_) ? "r_sole" : "l_sole", goals.header, goals.ball_position,
                                        goals.kick_direction);
  // TODO: handle when goal was not transformed
  tf2::convert(transformed_goal->first, ball_position_);
  tf2::convert(transformed_goal->second, kick_direction_);
  kick_direction_.normalize();
  kick_speed_ = goals.kick_speed;

  time_ = 0;

  /* Plan new splines according to new goal */
  calcSplines(is_left_kick_ ? goals.l_foot_pose : goals.r_foot_pose);
}

KickPositions KickEngine::update(double dt) {
  /* Only do an actual update when splines are present */
  // if (support_point_trajectories_ && flying_trajectories_) {
  KickPositions positions;
  /* Get should-be pose from planned splines (every axis) at current time */
  positions.support_point.x = support_point_spline_.x()->pos(time_);
  positions.support_point.y = support_point_spline_.x()->pos(time_);
  positions.flying_foot_pose = getCurrentPose(flying_foot_spline_);
  positions.is_left_kick = is_left_kick_;

  /* calculate if we want to use center-of-pressure in the current phase
   * use COP based support point only when the weight is on the support foot
   * while raising/lowering the foot, the weight is not completely on the support foot (that's why /2.0)*/
  if (time_ > params_.move_trunk_time + params_.raise_foot_time/2.0 &&
      time_ < phase_timings_.move_back + params_.lower_foot_time/2.0) {
    positions.cop_support_point = true;
  } else {
    positions.cop_support_point = false;
  }

  time_ += dt;

  /* Stabilize and return result */
  return positions;
}

geometry_msgs::PoseStamped KickEngine::getCurrentPose(bitbots_splines::PoseSpline spline) {
  geometry_msgs::PoseStamped foot_pose;
  foot_pose.header.frame_id = "l_sole"; // TODO
  foot_pose.header.stamp = ros::Time::now();
  foot_pose.pose = spline.getGeometryMsgPose(time_);
  return foot_pose;
}

void KickEngine::calcSplines(const geometry_msgs::Pose &flying_foot_pose) {
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
   *  - move trunk back
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
  double speed_yaw = tf2::getYaw(kick_direction_);
  tf2::Vector3 speed_vector(cos(speed_yaw), sin(speed_yaw), 0);

  /* Flying foot position */
  flying_foot_spline_.x()->addPoint(0, flying_foot_pose.position.x);
  flying_foot_spline_.x()->addPoint(phase_timings_.move_trunk, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.raise_foot, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.windup, windup_point_.x(), 0, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.kick, ball_position_.x(),
                                              speed_vector.x()*kick_speed_, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.move_back, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.lower_foot, 0);
  flying_foot_spline_.x()->addPoint(phase_timings_.move_trunk_back, 0);

  flying_foot_spline_.y()->addPoint(0, flying_foot_pose.position.y);
  flying_foot_spline_.y()->addPoint(phase_timings_.move_trunk, kick_foot_sign*params_.foot_distance);
  flying_foot_spline_.y()->addPoint(phase_timings_.raise_foot, kick_foot_sign*params_.foot_distance);
  flying_foot_spline_.y()->addPoint(phase_timings_.windup, windup_point_.y(), 0, 0);
  flying_foot_spline_.y()
      ->addPoint(phase_timings_.kick, ball_position_.y(), speed_vector.y()*kick_speed_, 0);
  flying_foot_spline_.y()->addPoint(phase_timings_.move_back, kick_foot_sign*params_.foot_distance);
  flying_foot_spline_.y()->addPoint(phase_timings_.lower_foot, kick_foot_sign*params_.foot_distance);
  flying_foot_spline_.y()->addPoint(phase_timings_.move_trunk_back, kick_foot_sign*params_.foot_distance);

  flying_foot_spline_.z()->addPoint(0, flying_foot_pose.position.z);
  flying_foot_spline_.z()->addPoint(phase_timings_.move_trunk, 0);
  flying_foot_spline_.z()->addPoint(phase_timings_.raise_foot, params_.foot_rise);
  flying_foot_spline_.z()->addPoint(phase_timings_.windup, params_.foot_rise);
  flying_foot_spline_.z()->addPoint(phase_timings_.kick, params_.foot_rise);
  flying_foot_spline_.z()->addPoint(phase_timings_.move_back, params_.foot_rise);
  flying_foot_spline_.z()->addPoint(phase_timings_.lower_foot, 0.4*params_.foot_rise);
  flying_foot_spline_.z()->addPoint(phase_timings_.move_trunk_back, 0);

  /* Flying foot orientation */
  /* Construct a start_rotation as quaternion from Pose msg */
  tf2::Quaternion start_rotation(flying_foot_pose.orientation.x, flying_foot_pose.orientation.y,
                                 flying_foot_pose.orientation.z, flying_foot_pose.orientation.w);
  double start_r, start_p, start_y;
  tf2::Matrix3x3(start_rotation).getRPY(start_r, start_p, start_y);

  /* Also construct one for the target */
  tf2::Quaternion target_rotation(flying_foot_pose.orientation.x, flying_foot_pose.orientation.y,
                                  flying_foot_pose.orientation.z, flying_foot_pose.orientation.w);
  double target_r, target_p, target_y;
  tf2::Matrix3x3(target_rotation).getRPY(target_r, target_p, target_y);

  target_y = calcKickFootYaw();

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
  support_point_spline_.x()->addPoint(0, 0);
  support_point_spline_.x()->addPoint(phase_timings_.move_trunk, params_.stabilizing_point_x);
  support_point_spline_.x()->addPoint(phase_timings_.raise_foot, params_.stabilizing_point_x);
  support_point_spline_.x()->addPoint(phase_timings_.windup, params_.stabilizing_point_x);
  support_point_spline_.x()->addPoint(phase_timings_.kick, params_.stabilizing_point_x);
  support_point_spline_.x()->addPoint(phase_timings_.move_back, params_.stabilizing_point_x);
  support_point_spline_.x()->addPoint(phase_timings_.lower_foot, params_.stabilizing_point_x);
  support_point_spline_.x()->addPoint(phase_timings_.move_trunk_back, 0);

  support_point_spline_.y()->addPoint(0, kick_foot_sign*(params_.foot_distance/2.0));
  support_point_spline_.y()
      ->addPoint(phase_timings_.move_trunk, kick_foot_sign*(-params_.stabilizing_point_y));
  support_point_spline_.y()
      ->addPoint(phase_timings_.raise_foot, kick_foot_sign*(-params_.stabilizing_point_y));
  support_point_spline_.y()
      ->addPoint(phase_timings_.windup, kick_foot_sign*(-params_.stabilizing_point_y));
  support_point_spline_.y()
      ->addPoint(phase_timings_.kick, kick_foot_sign*(-params_.stabilizing_point_y));
  support_point_spline_.y()
      ->addPoint(phase_timings_.move_back, kick_foot_sign*(-params_.stabilizing_point_y));
  support_point_spline_.y()
      ->addPoint(phase_timings_.lower_foot, kick_foot_sign*(-params_.stabilizing_point_y));
  support_point_spline_.y()
      ->addPoint(phase_timings_.move_trunk_back, kick_foot_sign*(params_.foot_distance/2.0));
}

std::optional<std::pair<geometry_msgs::Point, geometry_msgs::Quaternion>> KickEngine::transformGoal(
    const std::string &support_foot_frame,
    const std_msgs::Header &header,
    const geometry_msgs::Vector3 &ball_position,
    const geometry_msgs::Quaternion &kick_direction) {
  /* construct stamped goals so that they can be transformed */ // TODO Extract this into own function because we do it multiple times
  geometry_msgs::PointStamped
      stamped_position;       // TODO Make KickGoal a point as well so we dont have to do transformations here
  stamped_position.point.x = ball_position.x;
  stamped_position.point.y = ball_position.y;
  stamped_position.point.z = ball_position.z;
  stamped_position.header = header;
  //stamped_position.vector = ball_position;
  geometry_msgs::QuaternionStamped stamped_direction;
  stamped_direction.header = header;
  stamped_direction.quaternion = kick_direction;

  /* do transform into support_foot frame */
  geometry_msgs::PointStamped transformed_position;
  geometry_msgs::QuaternionStamped transformed_direction;

  tf_buffer_.transform(stamped_position, transformed_position, support_foot_frame, ros::Duration(0.2));
  tf_buffer_.transform(stamped_direction, transformed_direction, support_foot_frame, ros::Duration(0.2));

  auto x = tf_buffer_.lookupTransform(support_foot_frame, header.frame_id, header.stamp, ros::Duration(0.2));

  return std::pair(transformed_position.point, transformed_direction.quaternion);
}

tf2::Vector3 KickEngine::calcKickWindupPoint() {
  /* retrieve yaw from kick_direction_ */
  double yaw = tf2::getYaw(kick_direction_);

  /* create a vector which points in the negative direction of kick_direction_ */
  tf2::Vector3 vec(cos(yaw), sin(yaw), 0);
  vec.normalize();

  /* take windup distance into account */
  vec *= -params_.kick_windup_distance;

  /* add the ball position because the windup point is in support_foot_frame and not ball_frame */
  vec += ball_position_;

  vec.setZ(params_.foot_rise);

  return vec;
}

bool KickEngine::calcIsLeftFootKicking(const std_msgs::Header &header,
                                       const geometry_msgs::Vector3 &ball_position,
                                       const geometry_msgs::Quaternion &kick_direction) {
  /* prepare variables with stamps */
  geometry_msgs::Vector3Stamped stamped_position;
  stamped_position.header = header;
  stamped_position.vector = ball_position;
  geometry_msgs::QuaternionStamped stamped_direction;
  stamped_direction.header = header;
  stamped_direction.quaternion = kick_direction;

  /* transform ball data into frame where we want to apply it */
  tf2::Stamped<tf2::Vector3> transformed_ball_position;
  tf_buffer_.transform(stamped_position, transformed_ball_position, "base_footprint", ros::Duration(0.2));
  tf2::Stamped<tf2::Quaternion> transformed_direction;
  tf_buffer_.transform(stamped_direction, transformed_direction, "base_footprint", ros::Duration(0.2));

  /*
   * check if ball is outside of an imaginary corridor
   * if it is not, we use a more fined grained criterion which takes kick_direction into account
   */
  if (transformed_ball_position.y() > params_.choose_foot_corridor_width/2)
    return true;
  else if (transformed_ball_position.y() < -params_.choose_foot_corridor_width/2)
    return false;

  /* use the more fine grained angle based criterion
   * angle_1 = angle between "forward" and "origin-to-ball-position"
   * angle_2 = yaw of kick_direction
   * angle_diff = difference between the two on which the decision happens
   */
  double angle_1 = transformed_ball_position.angle({1, 0, 0});
  angle_1 *= transformed_ball_position.y() < 0 ? -1 : 1;

  double angle_2 = tf2::getYaw(transformed_direction);
  double angle_diff = angle_2 - angle_1;

  ROS_INFO_STREAM("Choosing " << ((angle_diff < 0) ? "left" : "right") << " foot to kick");

  return angle_diff < 0;
}

double KickEngine::calcKickFootYaw() {
  double kick_roll_angle, kick_pitch_angle, kick_yaw_angle;
  tf2::Matrix3x3(kick_direction_).getRPY(kick_roll_angle, kick_pitch_angle, kick_yaw_angle);

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
  return int(time_/phase_timings_.move_trunk_back*100);
}

bitbots_splines::PoseSpline KickEngine::getSplines() const {
  return flying_foot_spline_;
}

KickPhase KickEngine::getPhase() const {
  if (time_==0)
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

void KickEngine::setParams(KickParams params) {
  params_ = params;
}

tf2::Vector3 KickEngine::getWindupPoint() {
  return windup_point_;
}

}
