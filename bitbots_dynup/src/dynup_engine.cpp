#include "bitbots_dynup/dynup_engine.h"

namespace bitbots_dynup {

DynupEngine::DynupEngine() : listener_(tf_buffer_) {}

void DynupEngine::reset() {
  time_ = 0;
  duration_ = 0;
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
  geometry_msgs::Point support_point;
  support_point.x = l_foot_pose.pose.position.x;
  support_point.y = l_foot_pose.pose.position.y - params_.foot_distance/2;
  support_point.z = l_foot_pose.pose.position.z;

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
void DynupEngine::initializeSplines(geometry_msgs::Pose pose, bitbots_splines::PoseSpline spline) {
  double time_start = 0.0;
  double r,p,y;
  tf2::Quaternion q;

  spline.x()->addPoint(time_start, pose.position.x);
  spline.y()->addPoint(time_start, pose.position.y);
  spline.z()->addPoint(time_start, pose.position.z);
  tf2::convert(pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  spline.roll()->addPoint(time_start, r);
  spline.pitch()->addPoint(time_start, p);
  spline.yaw()->addPoint(time_start, y);

}

void DynupEngine::calcFrontSplines() {
  /*
  calculates splines for front up
  */
  
  /*
   * hands to the side
   */
  double time = params_.time_hands_side;
  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, params_.arm_max_length - params_.arm_offset_y);
  l_hand_spline_.z()->addPoint(time, params_.arm_offset_z);
  l_hand_spline_.roll()->addPoint(time, M_PI/2);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, M_PI/2);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -params_.arm_max_length + params_.arm_offset_y);
  r_hand_spline_.z()->addPoint(time, params_.arm_offset_z);
  r_hand_spline_.roll()->addPoint(time, -M_PI/2);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, -M_PI/2);

  /*
   * pull legs to body
   */
  time += params_.time_foot_close;
  trunk_spline_.x()->addPoint(time, 0);
  trunk_spline_.y()->addPoint(time, params_.foot_distance / 2);
  trunk_spline_.z()->addPoint(time, params_.leg_min_length);
  trunk_spline_.roll()->addPoint(time, 0);
  trunk_spline_.pitch()->addPoint(time, 0);
  trunk_spline_.yaw()->addPoint(time, 0);
  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);


  /*
   * hands to the front
   */
  time += params_.time_hands_front;
  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, params_.arm_offset_y);
  l_hand_spline_.z()->addPoint(time, params_.arm_max_length + params_.arm_offset_z);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, -M_PI/2);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -params_.arm_offset_y);
  r_hand_spline_.z()->addPoint(time, params_.arm_max_length + params_.arm_offset_z);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, -M_PI/2);
  r_hand_spline_.yaw()->addPoint(time, 0);

  /*
   * Foot under body
   */
  time += params_.time_foot_ground;
  trunk_spline_.x()->addPoint(time, cos(90) * params_.leg_min_length);
  trunk_spline_.y()->addPoint(time, params_.foot_distance / 2);
  trunk_spline_.z()->addPoint(time, sin(90) * params_.leg_min_length);
  trunk_spline_.roll()->addPoint(time, 0);
  trunk_spline_.pitch()->addPoint(time, M_PI/2);
  trunk_spline_.yaw()->addPoint(time, 0);
  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);


  /*
   * Torso 45°
   */
  time += params_.time_torso_45;
  l_hand_spline_.x()->addPoint(time, params_.arm_max_length);
  l_hand_spline_.y()->addPoint(time, params_.arm_offset_y);
  l_hand_spline_.z()->addPoint(time, params_.arm_offset_z);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, params_.arm_max_length);
  r_hand_spline_.y()->addPoint(time, -params_.arm_offset_y);
  r_hand_spline_.z()->addPoint(time, params_.arm_offset_z);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, 0);

  /*
   * To Squat
   */
  time += params_.time_to_squat;
  l_hand_spline_.x()->addPoint(time + 0.5, 0);
  l_hand_spline_.y()->addPoint(time + 0.5, params_.arm_offset_y);
  l_hand_spline_.z()->addPoint(time + 0.5, -params_.arm_max_length +params_.arm_offset_z);
  l_hand_spline_.roll()->addPoint(time + 0.5, 0);
  l_hand_spline_.pitch()->addPoint(time + 0.5, M_PI/2);
  l_hand_spline_.yaw()->addPoint(time + 0.5, 0);
  r_hand_spline_.x()->addPoint(time + 0.5, 0);
  r_hand_spline_.y()->addPoint(time + 0.5, -params_.arm_offset_y);
  r_hand_spline_.z()->addPoint(time + 0.5, -params_.arm_max_length + params_.arm_offset_z);
  r_hand_spline_.roll()->addPoint(time + 0.5, 0);
  r_hand_spline_.pitch()->addPoint(time + 0.5, M_PI/2);
  r_hand_spline_.yaw()->addPoint(time + 0.5, 0);

  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);
  trunk_spline_.x()->addPoint(time, 0);
  trunk_spline_.y()->addPoint(time, params_.foot_distance / 2);
  trunk_spline_.z()->addPoint(time, params_.leg_min_length);
  trunk_spline_.roll()->addPoint(time, 0);
  trunk_spline_.pitch()->addPoint(time, 0);
  trunk_spline_.yaw()->addPoint(time, 0); 

  calcSquatSplines(time);
}

void DynupEngine::calcBackSplines() {
  /*
  calculates splines for back up
  */

  /*
   * pull legs to body
   */
  double time = params_.time_hands_down;
  l_hand_spline_.x()->addPoint(time, -params_.arm_max_length/2);
  l_hand_spline_.y()->addPoint(time, params_.arm_offset_y);
  l_hand_spline_.z()->addPoint(time, 0);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI/2);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, -params_.arm_max_length/2);
  r_hand_spline_.y()->addPoint(time, -params_.arm_offset_y);
  r_hand_spline_.z()->addPoint(time, 0);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI/2);
  r_hand_spline_.yaw()->addPoint(time, 0);

  trunk_spline_.x()->addPoint(time, 0);
  trunk_spline_.y()->addPoint(time, params_.foot_distance / 2);
  trunk_spline_.z()->addPoint(time, params_.leg_min_length);
  trunk_spline_.roll()->addPoint(time, 0);
  trunk_spline_.pitch()->addPoint(time, 0);
  trunk_spline_.yaw()->addPoint(time, 0);
  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);

  /*
   * hands to the back
   */
  time += params_.time_hands_back;
  l_hand_spline_.x()->addPoint(time, -params_.arm_max_length);
  l_hand_spline_.y()->addPoint(time, params_.arm_offset_y);
  l_hand_spline_.z()->addPoint(time, params_.arm_offset_z);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, -params_.arm_max_length);
  r_hand_spline_.y()->addPoint(time, -params_.arm_offset_y);
  r_hand_spline_.z()->addPoint(time, params_.arm_offset_z);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI);
  r_hand_spline_.yaw()->addPoint(time, 0);

  trunk_spline_.x()->addPoint(time, -cos(45) * params_.leg_min_length);
  trunk_spline_.y()->addPoint(time, params_.foot_distance / 2);
  trunk_spline_.z()->addPoint(time, -sin(45) * params_.leg_min_length);
  trunk_spline_.roll()->addPoint(time, 0);
  trunk_spline_.pitch()->addPoint(time, -M_PI/4);
  trunk_spline_.yaw()->addPoint(time, 0);
  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);

  /*
   * To squat
   */
  time += params_.time_to_squat;
  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, params_.arm_offset_y);
  l_hand_spline_.z()->addPoint(time, -params_.arm_max_length +params_.arm_offset_z);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI/2);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -params_.arm_offset_y);
  r_hand_spline_.z()->addPoint(time, -params_.arm_max_length + params_.arm_offset_z);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI/2);
  r_hand_spline_.yaw()->addPoint(time, 0);

  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);
  trunk_spline_.x()->addPoint(time, 0);
  trunk_spline_.y()->addPoint(time, params_.foot_distance / 2);
  trunk_spline_.z()->addPoint(time, params_.leg_min_length);
  trunk_spline_.roll()->addPoint(time, 0);
  trunk_spline_.pitch()->addPoint(time, 0);
  trunk_spline_.yaw()->addPoint(time, 0); 

  calcSquatSplines(time);
}

void DynupEngine::calcSquatSplines(double time) {

  // all positions relative to right foot
  // foot_trajectories_ are for left foot
  time += params_.rise_time;
  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);

  trunk_spline_.x()->addPoint(time, params_.trunk_x);
  trunk_spline_.y()->addPoint(time, params_.foot_distance / 2.0);
  trunk_spline_.z()->addPoint(time, params_.trunk_height);
  trunk_spline_.roll()->addPoint(time, 0);
  trunk_spline_.pitch()->addPoint(time, params_.trunk_pitch);
  trunk_spline_.yaw()->addPoint(time, 0);
}

void DynupEngine::setGoals(const DynupRequest &goals) {
  initializeSplines(goals.l_foot_pose, foot_spline_);
  initializeSplines(goals.trunk_pose, trunk_spline_);
  initializeSplines(goals.l_hand_pose, l_hand_spline_);
  initializeSplines(goals.r_hand_pose, r_hand_spline_);
  if(goals.front){
     duration_ = params_.time_hands_side + 
                 params_.time_foot_close + 
                 params_.time_hands_front + 
                 params_.time_foot_ground + 
                 params_.time_torso_45 + 
                 params_.time_to_squat + 
                 params_.rise_time;
     calcFrontSplines();
  }else{
     duration_ = params_.time_hands_down + 
                 params_.time_hands_back + 
                 params_.time_to_squat + 
                 params_.rise_time;
     calcBackSplines();
  }
}

int DynupEngine::getPercentDone() const {
  return int(time_ / duration_ * 100);
}

void DynupEngine::setParams(DynUpConfig params) {
  params_ = params;
}

}
