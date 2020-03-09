#include "bitbots_dynup/dynup_engine.h"

namespace bitbots_dynup {

DynupEngine::DynupEngine() : listener_(tf_buffer_) {}

void DynupEngine::init(double arm_max_length, double arm_offset_y, double arm_offset_z) {
  arm_max_length_ = arm_max_length;
  arm_offset_y_ = arm_offset_y - 0.1;
  arm_offset_z_ = arm_offset_z;
}

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

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;

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
  tf2::Transform l_foot_pose = foot_spline_.getTfTransform(time_);
  tf2::Transform r_foot_pose = r_foot_spline_.getTfTransform(time_);
  tf2::Transform l_hand_pose = l_hand_spline_.getTfTransform(time_);
  tf2::Transform r_hand_pose = r_hand_spline_.getTfTransform(time_);

  goals_.l_foot_goal_pose = l_foot_pose;
  goals_.r_foot_goal_pose = r_foot_pose;
  goals_.l_hand_goal_pose = l_hand_pose;
  goals_.r_hand_goal_pose = r_hand_pose;

  time_ += dt;

  return goals_;
}

//TODO: Simplify
void DynupEngine::initializeSplines(geometry_msgs::Pose l_hand_pose, geometry_msgs::Pose r_hand_pose, geometry_msgs::Pose l_foot_pose, geometry_msgs::Pose r_foot_pose) {
  double time_start = 0.0;
  double r,p,y;
  tf2::Quaternion q;

  l_hand_spline_.x()->addPoint(time_start, l_hand_pose.position.x);
  l_hand_spline_.y()->addPoint(time_start, l_hand_pose.position.y);
  l_hand_spline_.z()->addPoint(time_start, l_hand_pose.position.z);
  tf2::convert(l_hand_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  l_hand_spline_.roll()->addPoint(time_start, r);
  l_hand_spline_.pitch()->addPoint(time_start, p);
  l_hand_spline_.yaw()->addPoint(time_start, y);

  r_hand_spline_.x()->addPoint(time_start, r_hand_pose.position.x);
  r_hand_spline_.y()->addPoint(time_start, r_hand_pose.position.y);
  r_hand_spline_.z()->addPoint(time_start, r_hand_pose.position.z);
  tf2::convert(r_hand_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  r_hand_spline_.roll()->addPoint(time_start, r);
  r_hand_spline_.pitch()->addPoint(time_start, p);
  r_hand_spline_.yaw()->addPoint(time_start, y);

  foot_spline_.x()->addPoint(time_start, l_foot_pose.position.x);
  foot_spline_.y()->addPoint(time_start, l_foot_pose.position.y);
  foot_spline_.z()->addPoint(time_start, l_foot_pose.position.z);
  tf2::convert(l_foot_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  foot_spline_.roll()->addPoint(time_start, r);
  foot_spline_.pitch()->addPoint(time_start, p);
  foot_spline_.yaw()->addPoint(time_start, y);

  r_foot_spline_.x()->addPoint(time_start, r_foot_pose.position.x);
  r_foot_spline_.y()->addPoint(time_start, r_foot_pose.position.y);
  r_foot_spline_.z()->addPoint(time_start, r_foot_pose.position.z);
  tf2::convert(r_foot_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  r_foot_spline_.roll()->addPoint(time_start, r);
  r_foot_spline_.pitch()->addPoint(time_start, p);
  r_foot_spline_.yaw()->addPoint(time_start, y);

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
  l_hand_spline_.y()->addPoint(time, arm_max_length_ - arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, arm_offset_z_);
  l_hand_spline_.roll()->addPoint(time, M_PI/2);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, M_PI/2);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -arm_max_length_ + arm_offset_y_);
  r_hand_spline_.z()->addPoint(time, arm_offset_z_);
  r_hand_spline_.roll()->addPoint(time, -M_PI/2);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, -M_PI/2);

  /*
   * pull legs to body
   */
  time += params_.time_foot_close;
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
   * hands to the front
   */
  time += params_.time_hands_front;
  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, -arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, arm_max_length_ + arm_offset_z_);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, -M_PI/2);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, arm_offset_y_);
  r_hand_spline_.z()->addPoint(time, arm_max_length_ + arm_offset_z_);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, -M_PI/2);
  r_hand_spline_.yaw()->addPoint(time, 0);

  /*
   * Foot under body
   */
  time += params_.time_foot_ground;
  r_foot_spline_.x()->addPoint(time, sin(70) * params_.leg_min_length);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -cos(70) * params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, -M_PI * 70 /180);//70 degrees
  r_foot_spline_.yaw()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);


  /*
   * Torso 45°
   */
  time += params_.time_torso_45;
  l_hand_spline_.x()->addPoint(time, arm_max_length_);
  l_hand_spline_.y()->addPoint(time, -arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, arm_offset_z_);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, arm_max_length_);
  r_hand_spline_.y()->addPoint(time, arm_offset_y_);
  r_hand_spline_.z()->addPoint(time, arm_offset_z_);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, 0);

  /*
   * To Squat
   */
  time += params_.time_to_squat;
  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, -arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, -arm_max_length_ +arm_offset_z_);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI/2);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, arm_offset_y_);
  r_hand_spline_.z()->addPoint(time, -arm_max_length_ + arm_offset_z_);
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
  r_foot_spline_.pitch()->addPoint(time,0);
  r_foot_spline_.yaw()->addPoint(time, 0);

  /*
   * Wait to negate velocities.
   */
  time += params_.wait_in_squat;
  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);
  r_foot_spline_.x()->addPoint(time, params_.trunk_x);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, params_.trunk_pitch);
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
  l_hand_spline_.x()->addPoint(time, -arm_max_length_/2);
  l_hand_spline_.y()->addPoint(time, -arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, 0);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI/2);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, -arm_max_length_/2);
  r_hand_spline_.y()->addPoint(time, arm_offset_y_);
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
   * Foot under body
   */
  time += params_.time_foot_ground;
  r_foot_spline_.x()->addPoint(time, -sin(70) * params_.leg_min_length);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -cos(70) * params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, M_PI * 70 /180);//70 degrees
  r_foot_spline_.yaw()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);

  /*
   * To squat
   */
  time += params_.time_to_squat;
  l_hand_spline_.x()->addPoint(time, -arm_max_length_);
  l_hand_spline_.y()->addPoint(time, -arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, arm_offset_z_);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, -arm_max_length_);
  r_hand_spline_.y()->addPoint(time, arm_offset_y_);
  r_hand_spline_.z()->addPoint(time, arm_offset_z_);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI);
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
  r_foot_spline_.pitch()->addPoint(time,0);
  r_foot_spline_.yaw()->addPoint(time, 0);

  /*
   * Wait to negate velocities.
   */
  time += params_.wait_in_squat;
  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);
  r_foot_spline_.x()->addPoint(time, params_.trunk_x);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, params_.trunk_pitch);
  r_foot_spline_.yaw()->addPoint(time, 0);

  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, -arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, -arm_max_length_ +arm_offset_z_);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI/2);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, arm_offset_y_);
  r_hand_spline_.z()->addPoint(time, -arm_max_length_ + arm_offset_z_);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI/2);
  r_hand_spline_.yaw()->addPoint(time, 0);

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

  r_foot_spline_.x()->addPoint(time, params_.trunk_x * 2.0);//TODO: Calculate this from trunk_pitch
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2.0);
  r_foot_spline_.z()->addPoint(time, -params_.trunk_height - params_.trunk_x);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, params_.trunk_pitch);
  r_foot_spline_.yaw()->addPoint(time, 0);
}

void DynupEngine::setGoals(const DynupRequest &goals) {
  initializeSplines(goals.l_hand_pose, goals.r_hand_pose, goals.l_foot_pose, goals.r_foot_pose);
  if(goals.front){
     duration_ = params_.time_hands_side + 
                 params_.time_foot_close + 
                 params_.time_hands_front + 
                 params_.time_foot_ground + 
                 params_.time_torso_45 + 
                 params_.time_to_squat +
                 params_.wait_in_squat +
                 params_.rise_time;
     front_ = true;
     calcFrontSplines();
  }else{
     duration_ = params_.time_hands_down +
                 params_.time_foot_ground +
                 params_.time_to_squat +
                 params_.wait_in_squat +
                 params_.rise_time;
     front_ = false;
     calcBackSplines();
  }
}

int DynupEngine::getPercentDone() const {
  return int(time_ / duration_ * 100);
}

/*Calculates if we are at a point of the animation where stabilizing should be applied. */ //TODO: make this nice
bool DynupEngine::isStabilizingNeeded() const {
    return (front_ && time_ >= params_.time_hands_side + params_.time_foot_close + params_.time_hands_front +
                               params_.time_foot_ground + params_.time_torso_45) ||
           (!front_ && time_ >= params_.time_hands_down +
                                params_.time_hands_back);
}

bitbots_splines::PoseSpline DynupEngine::getRFootSplines() const {
  return r_foot_spline_;
}

bitbots_splines::PoseSpline DynupEngine::getLFootSplines() const {
    return foot_spline_;
}

bitbots_splines::PoseSpline DynupEngine::getRHandSplines() const {
    return r_hand_spline_;
}

bitbots_splines::PoseSpline DynupEngine::getLHandSplines() const {
    return l_hand_spline_;
}

void DynupEngine::setParams(DynUpConfig params) {
  params_ = params;
}

}
