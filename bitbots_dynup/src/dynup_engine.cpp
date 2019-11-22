#include "bitbots_dynup/dynup_engine.h"

namespace bitbots_dynup {

DynupEngine::DynupEngine() : listener_(tf_buffer_) {}

void DynupEngine::reset() {
  time_ = 0;
  duration_ = 0;
  r_foot_spline_ = bitbots_splines::PoseSpline();
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
  geometry_msgs::PoseStamped hand_pose = getCurrentPose(hand_spline_, "l_wrist");
  //geometry_msgs::PoseStamped l_hand_pose = get_current_pose(foot_trajectories_.value(), "l_hand");
  //geometry_msgs::PoseStamped r_hand_pose = get_current_pose(foot_trajectories_.value(), "r_hand");


  time_ += dt;
  geometry_msgs::Point support_point; //TODO


  /* Stabilize and return result */
  DynupResponse goals;
  goals.support_point = support_point;
  goals.l_foot_goal_pose = l_foot_pose;
  goals.trunk_goal_pose = trunk_pose;
  goals.l_hand_goal_pose = hand_pose;
  return goals;
}

geometry_msgs::PoseStamped DynupEngine::getCurrentPose(bitbots_splines::PoseSpline spline, std::string frame_id) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.stamp = ros::Time::now();
  pose.pose = spline.getGeometryMsgPose(time_);
  return pose;
}

void DynupEngine::calcFrontSplines(geometry_msgs::Pose foot_pose, geometry_msgs::Pose hand_pose) {

  //
  // TODO is this correct?
  //

void DynupEngine::calcFrontSplines() {
  /*
  calculates splines for front up
  */
  
  /*
   * hands to the side
   */

  double time_start = 0;

  // hand
  //geometry_msgs::Pose hand_pose; //TODO read actual pose

  hand_spline_.x()->addPoint(time_start, hand_pose.position.x);
  hand_spline_.y()->addPoint(time_start, hand_pose.position.y);
  hand_spline_.z()->addPoint(time_start, hand_pose.position.z);

  /* Construct a start_rotation as quaternion from Pose msg */
  tf2::Quaternion hand_start_rotation(hand_pose.orientation.x, hand_pose.orientation.y,
                                      hand_pose.orientation.z, hand_pose.orientation.w);
  double hand_start_r, hand_start_p, hand_start_y;
  tf2::Matrix3x3(hand_start_rotation).getRPY(hand_start_r, hand_start_p, hand_start_y);
  hand_spline_.roll()->addPoint(time_start, hand_start_r);
  hand_spline_.pitch()->addPoint(time_start, hand_start_p);
  hand_spline_.yaw()->addPoint(time_start, hand_start_y);

  // foot
  //geometry_msgs::Pose foot_pose; //TODO read actual pose
  foot_spline_.x()->addPoint(time_start, foot_pose.position.x);
  foot_spline_.y()->addPoint(time_start, foot_pose.position.y);
  foot_spline_.z()->addPoint(time_start, foot_pose.position.z);

  /* Construct a start_rotation as quaternion from Pose msg */
  tf2::Quaternion foot_start_rotation(foot_pose.orientation.x, foot_pose.orientation.y,
                                      foot_pose.orientation.z, foot_pose.orientation.w);
  double foot_start_r, foot_start_p, foot_start_y;
  tf2::Matrix3x3(foot_start_rotation).getRPY(foot_start_r, foot_start_p, foot_start_y);
  foot_spline_.roll()->addPoint(time_start, foot_start_r);
  foot_spline_.pitch()->addPoint(time_start, foot_start_p);
  foot_spline_.yaw()->addPoint(time_start, foot_start_y);

  /*
   * hands to the side
   */
  double time_hands_side = params_.time_hands_side;
  hand_spline_.x()->addPoint(time_hands_side, 0);
  hand_spline_.y()->addPoint(time_hands_side, params_.arm_max_length);
  hand_spline_.z()->addPoint(time_hands_side, 0);
  hand_spline_.roll()->addPoint(time_hands_side, 0);
  hand_spline_.pitch()->addPoint(time_hands_side, M_PI/2);
  hand_spline_.yaw()->addPoint(time_hands_side, 0);

  /*
   * pull legs to body
  */
  double time_foot_close = params_.time_foot_close;
  foot_spline_.x()->addPoint(time_foot_close, 0);
  foot_spline_.y()->addPoint(time_foot_close, 0);
  foot_spline_.z()->addPoint(time_foot_close, params_.leg_min_length);
  foot_spline_.roll()->addPoint(time_foot_close, 0);
  foot_spline_.pitch()->addPoint(time_foot_close, 0);
  foot_spline_.yaw()->addPoint(time_foot_close, 0);


  /*
   * hands to the front
   */
  double time_hands_front = params_.time_hands_front;
  hand_spline_.x()->addPoint(time_hands_front, 0);
  hand_spline_.y()->addPoint(time_hands_front, 0);
  hand_spline_.z()->addPoint(time_hands_front, params_.arm_max_length);
  hand_spline_.roll()->addPoint(time_hands_front, 0);
  hand_spline_.pitch()->addPoint(time_hands_front, M_PI);
  hand_spline_.yaw()->addPoint(time_hands_front, 0);

  /*
   * Foot under body
   */
  double time_foot_ground = params_.time_foot_ground;
  foot_spline_.x()->addPoint(time_foot_ground, 0);
  foot_spline_.y()->addPoint(time_foot_ground, 0);
  foot_spline_.z()->addPoint(time_foot_ground, params_.leg_min_length);
  foot_spline_.roll()->addPoint(time_foot_ground, 0);
  foot_spline_.pitch()->addPoint(time_foot_ground, M_PI);
  foot_spline_.yaw()->addPoint(time_foot_ground, 0);


  /*
   * Torso 45°
   */
  double time_torso_45 = params_.time_torso_45;
  hand_spline_.x()->addPoint(time_torso_45, params_.arm_max_length);
  hand_spline_.y()->addPoint(time_torso_45, 0);
  hand_spline_.z()->addPoint(time_torso_45, 0);
  hand_spline_.roll()->addPoint(time_torso_45, 0);
  hand_spline_.pitch()->addPoint(time_torso_45, 0);
  hand_spline_.yaw()->addPoint(time_torso_45, 0);

  foot_spline_.x()->addPoint(time_torso_45, 0);
  foot_spline_.y()->addPoint(time_torso_45, 0);
  foot_spline_.z()->addPoint(time_torso_45, params_.leg_min_length);
  foot_spline_.roll()->addPoint(time_torso_45, 0);
  foot_spline_.pitch()->addPoint(time_torso_45, M_PI);
  foot_spline_.yaw()->addPoint(time_torso_45, 0);

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
  r_foot_spline_.x()->addPoint(time, 0);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, 0);
  r_foot_spline_.yaw()->addPoint(time, 0); 

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

  r_foot_spline_.x()->addPoint(time, 0);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, 0);
  r_foot_spline_.yaw()->addPoint(time, 0);
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

  r_foot_spline_.x()->addPoint(time, cos(45) * params_.leg_min_length);
  r_foot_spline_.y()->addPoint(time, params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, sin(45) * params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, M_PI/4);
  r_foot_spline_.yaw()->addPoint(time, 0);
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
  r_foot_spline_.x()->addPoint(time, 0);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, 0);
  r_foot_spline_.yaw()->addPoint(time, 0); 

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

  r_foot_spline_.x()->addPoint(time, -params_.trunk_x);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2.0);
  r_foot_spline_.z()->addPoint(time, -params_.trunk_height);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, -params_.trunk_pitch);
  r_foot_spline_.yaw()->addPoint(time, 0);
}

void DynupEngine::setGoals(const DynupRequest &goals) {
  if(goals.front){
  //TODO decide on which side we are lying on
     calcFrontSplines(goals.l_foot_pose, goals.l_hand_pose);
  }else{
     duration_ = params_.time_hands_down + 
                 params_.time_hands_back + 
                 params_.time_to_squat + 
                 params_.rise_time;
     calcBackSplines();
  }
  calcSquatSplines(goals.l_foot_pose, goals.trunk_pose);
}

int DynupEngine::getPercentDone() const {
  return int(time_ / duration_ * 100);
}

void DynupEngine::setParams(DynUpConfig params) {
  params_ = params;
}

}
