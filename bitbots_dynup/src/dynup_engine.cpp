#include "bitbots_dynup/dynup_engine.h"

namespace bitbots_dynup {

DynupEngine::DynupEngine() : listener_(tf_buffer_) {}

void DynupEngine::reset() {
  time_ = 0;
  trunk_spline_ = bitbots_splines::PoseSpline();
  l_hand_spline_ = bitbots_splines::PoseSpline();
  r_hand_spline_ = bitbots_splines::PoseSpline();
  foot_spline_ = bitbots_splines::PoseSpline();
}

DynupResponse DynupEngine::update(double dt) {
  // TODO what happens when splines for foot and trunk are not present?
  /* Get should-be pose from planned splines (every axis) at current time */
  geometry_msgs::PoseStamped l_foot_pose = getCurrentPose(foot_spline_, "l_sole");
  geometry_msgs::PoseStamped trunk_pose = getCurrentPose(trunk_spline_, "torso");
  geometry_msgs::PoseStamped l_hand_pose = getCurrentPose(l_hand_spline_, "l_wrist");
  geometry_msgs::PoseStamped r_hand_pose = getCurrentPose(r_hand_spline_, "r_wrist");

  time_ += dt;
  //TODO support point between feet
  geometry_msgs::Point support_point;
  /* Stabilize and return result */
  DynupResponse goals;
  goals.support_point = support_point;
  goals.l_foot_goal_pose = l_foot_pose;
  goals.trunk_goal_pose = trunk_pose;
  goals.l_hand_goal_pose = l_hand_pose;
  goals.r_hand_goal_pose = r_hand_pose;
  return goals;
}

geometry_msgs::PoseStamped DynupEngine::getCurrentPose(bitbots_splines::PoseSpline spline, std::string frame_id) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = ros::Time::now();
  pose.pose = spline.getGeometryMsgPose(time_);
  return pose;
}

void DynupEngine::initializeSplines(geometry_msgs::Pose foot_pose, geometry_msgs::Pose trunk_pose, geometry_msgs::Pose
                                      l_hand_pose, geometry_msgs::Pose r_hand_pose) {

  double time_start = 0;
  double r, p, y;
  tf2::Quaternion q;

  //left hand
  l_hand_spline_.x()->addPoint(time_start, l_hand_pose.position.x);
  l_hand_spline_.y()->addPoint(time_start, l_hand_pose.position.y);
  l_hand_spline_.z()->addPoint(time_start, l_hand_pose.position.z);
  tf2::convert(l_hand_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  l_hand_spline_.roll()->addPoint(time_start, r);
  l_hand_spline_.pitch()->addPoint(time_start, p);
  l_hand_spline_.yaw()->addPoint(time_start, y);
  
  //right hand
  r_hand_spline_.x()->addPoint(time_start, r_hand_pose.position.x);
  r_hand_spline_.y()->addPoint(time_start, r_hand_pose.position.y);
  r_hand_spline_.z()->addPoint(time_start, r_hand_pose.position.z);
  tf2::convert(r_hand_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  r_hand_spline_.roll()->addPoint(time_start, r);
  r_hand_spline_.pitch()->addPoint(time_start, p);
  r_hand_spline_.yaw()->addPoint(time_start, y);

  // foot
  foot_spline_.x()->addPoint(time_start, foot_pose.position.x);
  foot_spline_.y()->addPoint(time_start, foot_pose.position.y);
  foot_spline_.z()->addPoint(time_start, foot_pose.position.z);
  tf2::convert(foot_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  foot_spline_.roll()->addPoint(time_start, r);
  foot_spline_.pitch()->addPoint(time_start, p);
  foot_spline_.yaw()->addPoint(time_start, y);

  // trunk_spline_ are for trunk (relative to right foot)
  trunk_spline_.x()->addPoint(0, trunk_pose.position.x);
  trunk_spline_.y()->addPoint(0, trunk_pose.position.y);
  trunk_spline_.z()->addPoint(0, trunk_pose.position.z);
  tf2::convert(trunk_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  trunk_spline_.roll()->addPoint(0, r);
  trunk_spline_.pitch()->addPoint(0, p);
  trunk_spline_.yaw()->addPoint(0, y);
}

void DynupEngine::calcFrontSplines() {

  //
  // TODO is this correct?
  //

  /*
  calculates splines for front up
  */

  /*
   * start spline point with current poses
   */

  double foot_rotation = 0.2; //TODO: make param
  double arm_offset_y = 0.085;
  double arm_offset_z = 0.1;

  
  /*
   * hands to the side
   */
  double time_hands_side = params_.time_hands_side;
  l_hand_spline_.x()->addPoint(time_hands_side, 0);
  l_hand_spline_.y()->addPoint(time_hands_side, params_.arm_max_length - arm_offset_y);
  l_hand_spline_.z()->addPoint(time_hands_side, arm_offset_z);
  l_hand_spline_.roll()->addPoint(time_hands_side, M_PI/2);
  l_hand_spline_.pitch()->addPoint(time_hands_side, 0);
  l_hand_spline_.yaw()->addPoint(time_hands_side, M_PI/2);
  r_hand_spline_.x()->addPoint(time_hands_side, 0);
  r_hand_spline_.y()->addPoint(time_hands_side, -params_.arm_max_length + arm_offset_y);
  r_hand_spline_.z()->addPoint(time_hands_side, arm_offset_z);
  r_hand_spline_.roll()->addPoint(time_hands_side, -M_PI/2);
  r_hand_spline_.pitch()->addPoint(time_hands_side, 0);
  r_hand_spline_.yaw()->addPoint(time_hands_side, -M_PI/2);

  /*
   * pull legs to body
   */
  double time_foot_close = params_.time_foot_close;
  trunk_spline_.x()->addPoint(time_foot_close, 0);
  trunk_spline_.y()->addPoint(time_foot_close, 0);
  trunk_spline_.z()->addPoint(time_foot_close, params_.leg_min_length);
  trunk_spline_.roll()->addPoint(time_foot_close, 0);
  trunk_spline_.pitch()->addPoint(time_foot_close, 0);
  trunk_spline_.yaw()->addPoint(time_foot_close, 0);
  foot_spline_.x()->addPoint(time_foot_close, 0);
  foot_spline_.y()->addPoint(time_foot_close, params_.foot_distance);
  foot_spline_.z()->addPoint(time_foot_close, 0);
  foot_spline_.roll()->addPoint(time_foot_close, 0);
  foot_spline_.pitch()->addPoint(time_foot_close, 0);
  foot_spline_.yaw()->addPoint(time_foot_close, 0);


  /*
   * hands to the front
   */
  double time_hands_front = params_.time_hands_front;
  l_hand_spline_.x()->addPoint(time_hands_front, 0);
  l_hand_spline_.y()->addPoint(time_hands_front, arm_offset_y);
  l_hand_spline_.z()->addPoint(time_hands_front, params_.arm_max_length + arm_offset_z);
  l_hand_spline_.roll()->addPoint(time_hands_front, 0);
  l_hand_spline_.pitch()->addPoint(time_hands_front, -M_PI/2);
  l_hand_spline_.yaw()->addPoint(time_hands_front, 0);
  r_hand_spline_.x()->addPoint(time_hands_front, 0);
  r_hand_spline_.y()->addPoint(time_hands_front, -arm_offset_y);
  r_hand_spline_.z()->addPoint(time_hands_front, params_.arm_max_length + arm_offset_z);
  r_hand_spline_.roll()->addPoint(time_hands_front, 0);
  r_hand_spline_.pitch()->addPoint(time_hands_front, -M_PI/2);
  r_hand_spline_.yaw()->addPoint(time_hands_front, 0);

  /*
   * Foot under body
   */
  double time_foot_ground = params_.time_foot_ground;
  trunk_spline_.x()->addPoint(time_foot_ground, -params_.leg_min_length);
  trunk_spline_.y()->addPoint(time_foot_ground, 0);
  trunk_spline_.z()->addPoint(time_foot_ground, 0);
  trunk_spline_.roll()->addPoint(time_foot_ground, 0);
  trunk_spline_.pitch()->addPoint(time_foot_ground, M_PI/2 - foot_rotation);
  trunk_spline_.yaw()->addPoint(time_foot_ground, 0);
  foot_spline_.x()->addPoint(time_foot_ground, 0);
  foot_spline_.y()->addPoint(time_foot_ground, params_.foot_distance);
  foot_spline_.z()->addPoint(time_foot_ground, 0);
  foot_spline_.roll()->addPoint(time_foot_ground, 0);
  foot_spline_.pitch()->addPoint(time_foot_ground, 0);
  foot_spline_.yaw()->addPoint(time_foot_ground, 0);


  /*
   * Torso 45°
   */
  double time_torso_45 = params_.time_torso_45;
  l_hand_spline_.x()->addPoint(time_torso_45, params_.arm_max_length);
  l_hand_spline_.y()->addPoint(time_torso_45, arm_offset_y);
  l_hand_spline_.z()->addPoint(time_torso_45, arm_offset_z);
  l_hand_spline_.roll()->addPoint(time_torso_45, 0);
  l_hand_spline_.pitch()->addPoint(time_torso_45, 0);
  l_hand_spline_.yaw()->addPoint(time_torso_45, 0);
  r_hand_spline_.x()->addPoint(time_torso_45, params_.arm_max_length);
  r_hand_spline_.y()->addPoint(time_torso_45, -arm_offset_y);
  r_hand_spline_.z()->addPoint(time_torso_45, arm_offset_z);
  r_hand_spline_.roll()->addPoint(time_torso_45, 0);
  r_hand_spline_.pitch()->addPoint(time_torso_45, 0);
  r_hand_spline_.yaw()->addPoint(time_torso_45, 0);

  /*
   * To Squat
   */
  double time_to_squat = params_.time_to_squat;
  l_hand_spline_.x()->addPoint(time_to_squat, 0);
  l_hand_spline_.y()->addPoint(time_to_squat, arm_offset_y);
  l_hand_spline_.z()->addPoint(time_to_squat, -params_.arm_max_length +arm_offset_z);
  l_hand_spline_.roll()->addPoint(time_to_squat, 0);
  l_hand_spline_.pitch()->addPoint(time_to_squat, M_PI/2);
  l_hand_spline_.yaw()->addPoint(time_to_squat, 0);
  r_hand_spline_.x()->addPoint(time_to_squat, 0);
  r_hand_spline_.y()->addPoint(time_to_squat, -arm_offset_y);
  r_hand_spline_.z()->addPoint(time_to_squat, -params_.arm_max_length + arm_offset_z);
  r_hand_spline_.roll()->addPoint(time_to_squat, 0);
  r_hand_spline_.pitch()->addPoint(time_to_squat, M_PI/2);
  r_hand_spline_.yaw()->addPoint(time_to_squat, 0);

  foot_spline_.x()->addPoint(time_to_squat, 0);
  foot_spline_.y()->addPoint(time_to_squat, params_.foot_distance);
  foot_spline_.z()->addPoint(time_to_squat, 0);
  foot_spline_.roll()->addPoint(time_to_squat, 0);
  foot_spline_.pitch()->addPoint(time_to_squat, 0);
  foot_spline_.yaw()->addPoint(time_to_squat, 0);
  trunk_spline_.x()->addPoint(time_to_squat, 0);
  trunk_spline_.y()->addPoint(time_to_squat, 0);
  trunk_spline_.z()->addPoint(time_to_squat, params_.leg_min_length);
  trunk_spline_.roll()->addPoint(time_to_squat, 0);
  trunk_spline_.pitch()->addPoint(time_to_squat, 0);
  trunk_spline_.yaw()->addPoint(time_to_squat, 0); 
}

void DynupEngine::calcBackSplines() {

  //TODO from back to squat

}

void DynupEngine::calcSquatSplines() {

  // current position as first spline point
  // all positions relative to right foot

  // foot_trajectories_ are for left foot
  foot_spline_.x()->addPoint(params_.rise_time, 0);
  foot_spline_.y()->addPoint(params_.rise_time, params_.foot_distance);
  foot_spline_.z()->addPoint(params_.rise_time, 0);
  foot_spline_.roll()->addPoint(params_.rise_time, 0);
  foot_spline_.pitch()->addPoint(params_.rise_time, 0);
  foot_spline_.yaw()->addPoint(params_.rise_time, 0);

  trunk_spline_.x()->addPoint(params_.rise_time, params_.trunk_x);
  trunk_spline_.y()->addPoint(params_.rise_time, params_.foot_distance / 2.0);
  trunk_spline_.z()->addPoint(params_.rise_time, params_.trunk_height);
  trunk_spline_.roll()->addPoint(params_.rise_time, 0);
  trunk_spline_.pitch()->addPoint(params_.rise_time, params_.trunk_pitch);
  trunk_spline_.yaw()->addPoint(params_.rise_time, 0);
}

void DynupEngine::setGoals(const DynupRequest &goals) {
  initializeSplines(goals.l_foot_pose, goals.trunk_pose, goals.l_hand_pose, goals.r_hand_pose);
  if(goals.front){
  //TODO decide on which side we are lying on
     calcFrontSplines();
  }else{
     calcBackSplines();
  }
  calcSquatSplines();
}

int DynupEngine::getPercentDone() const {
  double duration = params_.rise_time;
  return int(time_ / duration * 100);
}

void DynupEngine::setParams(DynUpParams params) {
  params_ = params;
}

}
