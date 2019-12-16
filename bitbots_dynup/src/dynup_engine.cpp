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

void DynupEngine::publishDebug(ros::Publisher debug_publisher) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.header.frame_id = "/base_link";
  marker.type = visualization_msgs::Marker::SPHERE;

  marker.pose.position.x = goals_.support_point.x;
  marker.pose.position.y = goals_.support_point.y;
  marker.pose.position.z = goals_.support_point.z;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 1.0;
  marker.color.a = 1.0;

  debug_publisher.publish(marker);
  return;
}

DynupResponse DynupEngine::update(double dt) {
  // TODO what happens when splines for foot and trunk are not present?
  /* Get should-be pose from planned splines (every axis) at current time */
  geometry_msgs::PoseStamped l_foot_pose = getCurrentPose(foot_spline_, "l_sole");
  geometry_msgs::PoseStamped trunk_pose = getCurrentPose(trunk_spline_, "torso");
  geometry_msgs::PoseStamped l_hand_pose = getCurrentPose(l_hand_spline_, "l_wrist");
  geometry_msgs::PoseStamped r_hand_pose = getCurrentPose(r_hand_spline_, "r_wrist");

  time_ += dt;
  geometry_msgs::Point support_point; //TODO


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
  trunk_spline_.y()->addPoint(time_foot_close, params_.foot_distance / 2);
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
  trunk_spline_.y()->addPoint(time_foot_ground, params_.foot_distance / 2);
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
  trunk_spline_.y()->addPoint(time_to_squat, params_.foot_distance / 2);
  trunk_spline_.z()->addPoint(time_to_squat, params_.leg_min_length);
  trunk_spline_.roll()->addPoint(time_to_squat, 0);
  trunk_spline_.pitch()->addPoint(time_to_squat, 0);
  trunk_spline_.yaw()->addPoint(time_to_squat, 0); 
}

void DynupEngine::calcBackSplines() {
  /*
  calculates splines for back up
  */

  double foot_rotation = 0.2; //TODO: make param
  double arm_offset_y = 0.085;
  double arm_offset_z = 0.1;


  /*
   * pull legs to body
   */
  double time_hands_back = 1.0;
  l_hand_spline_.x()->addPoint(time_hands_back, -params_.arm_max_length/2);
  l_hand_spline_.y()->addPoint(time_hands_back, arm_offset_y);
  l_hand_spline_.z()->addPoint(time_hands_back, 0);
  l_hand_spline_.roll()->addPoint(time_hands_back, 0);
  l_hand_spline_.pitch()->addPoint(time_hands_back, -M_PI/2);
  l_hand_spline_.yaw()->addPoint(time_hands_back, 0);
  r_hand_spline_.x()->addPoint(time_hands_back, -params_.arm_max_length/2);
  r_hand_spline_.y()->addPoint(time_hands_back, -arm_offset_y);
  r_hand_spline_.z()->addPoint(time_hands_back, 0);
  r_hand_spline_.roll()->addPoint(time_hands_back, 0);
  r_hand_spline_.pitch()->addPoint(time_hands_back, -M_PI/2);
  r_hand_spline_.yaw()->addPoint(time_hands_back, 0);

  trunk_spline_.x()->addPoint(time_hands_back, 0);
  trunk_spline_.y()->addPoint(time_hands_back, params_.foot_distance / 2);
  trunk_spline_.z()->addPoint(time_hands_back, params_.leg_min_length);
  trunk_spline_.roll()->addPoint(time_hands_back, 0);
  trunk_spline_.pitch()->addPoint(time_hands_back, 0);
  trunk_spline_.yaw()->addPoint(time_hands_back, 0);
  foot_spline_.x()->addPoint(time_hands_back, 0);
  foot_spline_.y()->addPoint(time_hands_back, params_.foot_distance);
  foot_spline_.z()->addPoint(time_hands_back, 0);
  foot_spline_.roll()->addPoint(time_hands_back, 0);
  foot_spline_.pitch()->addPoint(time_hands_back, 0);
  foot_spline_.yaw()->addPoint(time_hands_back, 0);
}

void DynupEngine::calcSquatSplines() {

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
  initializeSplines(goals.l_foot_pose, foot_spline_);
  initializeSplines(goals.trunk_pose, trunk_spline_);
  initializeSplines(goals.l_hand_pose, l_hand_spline_);
  initializeSplines(goals.r_hand_pose, r_hand_spline_);
  if(goals.front){
  //TODO decide on which side we are lying on
     calcFrontSplines();
  }else{
     duration_ = params_.time_hands_down + 
                 params_.time_hands_back + 
                 params_.time_to_squat + 
                 params_.rise_time;
     calcBackSplines();
  }
  calcSquatSplines();
}

int DynupEngine::getPercentDone() const {
  return int(time_ / duration_ * 100);
}

void DynupEngine::setParams(DynUpConfig params) {
  params_ = params;
}

}
