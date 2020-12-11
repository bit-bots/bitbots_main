#include "bitbots_dynup/dynup_engine.h"
#include <ros/ros.h>
#include <ros/console.h>

namespace bitbots_dynup {

DynupEngine::DynupEngine() : listener_(tf_buffer_) {}

void DynupEngine::init(double arm_max_length, double arm_offset_y, double arm_offset_z) {
  arm_max_length_ = arm_max_length;
  //this are just the offsets to the shoulder, we need to apply some additional offset to prevent collisions
  shoulder_offset_y_ = arm_offset_y;
  arm_offset_y_ = shoulder_offset_y_ + params_.arm_side_offset;
  arm_offset_z_ = arm_offset_z;
  ros::NodeHandle nh;
  pub_engine_debug_ = nh.advertise<bitbots_dynup::DynupEngineDebug>("dynup_engine_debug", 1);
}

void DynupEngine::reset() {
  time_ = 0;
  duration_ = 0;
  r_foot_spline_ = bitbots_splines::PoseSpline();
  l_hand_spline_ = bitbots_splines::PoseSpline();
  r_hand_spline_ = bitbots_splines::PoseSpline();
  // todo naming is suboptimal. is this the left_foot_spline?
  foot_spline_ = bitbots_splines::PoseSpline();
}

void DynupEngine::publishDebug() {
    if (pub_engine_debug_.getNumSubscribers() == 0) {
        return;
    }

    bitbots_dynup::DynupEngineDebug msg;
    msg.header.stamp = ros::Time::now();

    msg.time = time_;
    if(time_ < params_.time_hands_side){
        msg.state_number = 0;
    }else if(time_ < params_.time_hands_side + params_.time_foot_close){
        msg.state_number = 1;
    }else if(time_ < params_.time_hands_side + params_.time_foot_close + params_.time_hands_front){
        msg.state_number = 2;
    }else if(time_ < params_.time_hands_side + params_.time_foot_close + params_.time_hands_front + params_.time_foot_ground){
        msg.state_number = 3;
    }else if(time_ < params_.time_hands_side + params_.time_foot_close + params_.time_hands_front + params_.time_foot_ground + params_.time_torso_45){
        msg.state_number = 4;
    }else if(time_ < params_.time_hands_side + params_.time_foot_close + params_.time_hands_front + params_.time_foot_ground + params_.time_torso_45 + params_.time_to_squat){
        msg.state_number = 5;
    }else {
        msg.state_number = 6;
    }

    geometry_msgs::Pose l_arm_pose;
    tf2::toMsg(goals_.l_hand_goal_pose, l_arm_pose);
    msg.l_arm_pose = l_arm_pose;
    geometry_msgs::Pose r_arm_pose;
    tf2::toMsg(goals_.r_hand_goal_pose, r_arm_pose);
    msg.r_arm_pose = r_arm_pose;
    geometry_msgs::Pose l_leg_pose;
    tf2::toMsg(goals_.l_foot_goal_pose, l_leg_pose);
    msg.l_leg_pose = l_leg_pose;
    geometry_msgs::Pose r_leg_pose;
    tf2::toMsg(goals_.r_foot_goal_pose, r_leg_pose);
    msg.r_leg_pose = r_leg_pose;

    pub_engine_debug_.publish(msg);
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

  publishDebug();

  time_ += dt;

  return goals_;
}

//TODO: Simplify
void DynupEngine::initializeSplines(geometry_msgs::Pose l_hand_pose, geometry_msgs::Pose r_hand_pose, geometry_msgs::Pose l_foot_pose, geometry_msgs::Pose r_foot_pose) {
  double r,p,y;
  tf2::Quaternion q;

  l_hand_spline_.x()->addPoint(0.0, l_hand_pose.position.x);
  l_hand_spline_.y()->addPoint(0.0, l_hand_pose.position.y);
  l_hand_spline_.z()->addPoint(0.0, l_hand_pose.position.z);
  tf2::convert(l_hand_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  l_hand_spline_.roll()->addPoint(0.0, r);
  l_hand_spline_.pitch()->addPoint(0.0, p);
  l_hand_spline_.yaw()->addPoint(0.0, y);

  r_hand_spline_.x()->addPoint(0.0, r_hand_pose.position.x);
  r_hand_spline_.y()->addPoint(0.0, r_hand_pose.position.y);
  r_hand_spline_.z()->addPoint(0.0, r_hand_pose.position.z);
  tf2::convert(r_hand_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  r_hand_spline_.roll()->addPoint(0.0, r);
  r_hand_spline_.pitch()->addPoint(0.0, p);
  r_hand_spline_.yaw()->addPoint(0.0, y);

  foot_spline_.x()->addPoint(0.0, l_foot_pose.position.x);
  foot_spline_.y()->addPoint(0.0, l_foot_pose.position.y);
  foot_spline_.z()->addPoint(0.0, l_foot_pose.position.z);
  tf2::convert(l_foot_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  foot_spline_.roll()->addPoint(0.0, r);
  foot_spline_.pitch()->addPoint(0.0, p);
  foot_spline_.yaw()->addPoint(0.0, y);

  r_foot_spline_.x()->addPoint(0.0, r_foot_pose.position.x);
  r_foot_spline_.y()->addPoint(0.0, r_foot_pose.position.y);
  r_foot_spline_.z()->addPoint(0.0, r_foot_pose.position.z);
  tf2::convert(r_foot_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  r_foot_spline_.roll()->addPoint(0.0, r);
  r_foot_spline_.pitch()->addPoint(0.0, p);
  r_foot_spline_.yaw()->addPoint(0.0, y);
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
  l_hand_spline_.y()->addPoint(time, arm_max_length_ + arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, arm_offset_z_);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, M_PI/2);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -arm_max_length_ - arm_offset_y_);
  r_hand_spline_.z()->addPoint(time, arm_offset_z_);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, -M_PI/2);

  time += 0.05; //TODO: promote to variable as soon as this works.
    l_hand_spline_.x()->addPoint(time, 0);
    l_hand_spline_.y()->addPoint(time, arm_max_length_ + arm_offset_y_);
    l_hand_spline_.z()->addPoint(time, arm_offset_z_);
    l_hand_spline_.roll()->addPoint(time, -M_PI/2);
    l_hand_spline_.pitch()->addPoint(time, 0);
    l_hand_spline_.yaw()->addPoint(time, M_PI/2);
    r_hand_spline_.x()->addPoint(time, 0);
    r_hand_spline_.y()->addPoint(time, -arm_max_length_ - arm_offset_y_);
    r_hand_spline_.z()->addPoint(time, arm_offset_z_);
    r_hand_spline_.roll()->addPoint(time, M_PI/2);
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
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -arm_offset_y_);
  r_hand_spline_.z()->addPoint(time, arm_max_length_ + arm_offset_z_);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, -M_PI/2);
  r_hand_spline_.yaw()->addPoint(time, 0);
  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, arm_max_length_ + arm_offset_z_);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, -M_PI/2);
  l_hand_spline_.yaw()->addPoint(time, 0);



  /*
   * Foot under body
   */
  time += params_.time_foot_ground;
  r_foot_spline_.x()->addPoint(time, sin(M_PI * -params_.max_leg_angle /180) * -params_.leg_min_length);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, cos(M_PI * -params_.max_leg_angle /180) * -params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, -M_PI * params_.max_leg_angle /180);
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
  r_hand_spline_.x()->addPoint(time, arm_max_length_);
  r_hand_spline_.y()->addPoint(time, -arm_offset_y_);
  r_hand_spline_.z()->addPoint(time, arm_offset_z_);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, 0);
  l_hand_spline_.x()->addPoint(time, arm_max_length_);
  l_hand_spline_.y()->addPoint(time, arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, arm_offset_z_);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, 0);

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

  /*
   * To Squat
   */
  time += params_.time_to_squat;
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -arm_offset_y_);
  r_hand_spline_.z()->addPoint(time, -arm_max_length_ + arm_offset_z_);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI/2);
  r_hand_spline_.yaw()->addPoint(time, 0);
  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, -arm_max_length_ + arm_offset_z_);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI/2);
  l_hand_spline_.yaw()->addPoint(time, 0);

  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);
  r_foot_spline_.x()->addPoint(time, sin(M_PI * params_.trunk_overshoot_angle_front /180) * -params_.leg_min_length);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, cos(M_PI * params_.trunk_overshoot_angle_front /180) * -params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, M_PI * params_.trunk_overshoot_angle_front /180);
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
  double time = params_.time_legs_close;
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

    l_hand_spline_.x()->addPoint(time, -0.15);
    l_hand_spline_.y()->addPoint(time, -arm_offset_y_);
    l_hand_spline_.z()->addPoint(time, arm_offset_z_ - 0.15);//TODO: Hacky hack, not actual position
    l_hand_spline_.roll()->addPoint(time, 0);
    l_hand_spline_.pitch()->addPoint(time, M_PI/2);
    l_hand_spline_.yaw()->addPoint(time, 0);
    r_hand_spline_.x()->addPoint(time, -0.15);
    r_hand_spline_.y()->addPoint(time, arm_offset_y_);
    r_hand_spline_.z()->addPoint(time, arm_offset_z_ - 0.1);
    r_hand_spline_.roll()->addPoint(time, 0);
    r_hand_spline_.pitch()->addPoint(time, M_PI/2);
    r_hand_spline_.yaw()->addPoint(time, 0);

  /*
   * Foot under body
   */
  time += params_.time_foot_ground;
    l_hand_spline_.x()->addPoint(time, -sqrt(2* pow(arm_max_length_/2, 2)));
    l_hand_spline_.y()->addPoint(time, -arm_offset_y_);
    l_hand_spline_.z()->addPoint(time, arm_offset_z_+0.28); //TODO: Also hacky hack, forcing elbow angle
    l_hand_spline_.roll()->addPoint(time, 0);
    l_hand_spline_.pitch()->addPoint(time, M_PI*0.6);
    l_hand_spline_.yaw()->addPoint(time, 0);
    r_hand_spline_.x()->addPoint(time, -sqrt(2* pow(arm_max_length_/2, 2)));
    r_hand_spline_.y()->addPoint(time, arm_offset_y_ );
    r_hand_spline_.z()->addPoint(time, arm_offset_z_+0.28);
    r_hand_spline_.roll()->addPoint(time, 0);
    r_hand_spline_.pitch()->addPoint(time, M_PI*0.6);
    r_hand_spline_.yaw()->addPoint(time, 0);

  r_foot_spline_.x()->addPoint(time, -sin(1.22173) * params_.leg_min_length);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -cos(1.22173) * params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, M_PI * 70 /180);//70 degrees
  r_foot_spline_.yaw()->addPoint(time, 0);
  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);

  /*
   * Halfway
   */
  time += params_.time_squat_push;
  l_hand_spline_.x()->addPoint(time, -sqrt(2* pow(arm_max_length_/2, 2)));
  l_hand_spline_.y()->addPoint(time, -arm_offset_y_);
  l_hand_spline_.z()->addPoint(time, arm_offset_z_+0.28); //TODO: Also hacky hack, forcing elbow angle
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI*0.9);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, -sqrt(2* pow(arm_max_length_/2, 2)));
  r_hand_spline_.y()->addPoint(time, arm_offset_y_ );
  r_hand_spline_.z()->addPoint(time, arm_offset_z_+0.28);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI*0.9);
  r_hand_spline_.yaw()->addPoint(time, 0);


  r_foot_spline_.x()->addPoint(time, -sin(0.872665) * params_.leg_min_length);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -cos(0.872665) * params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, M_PI * 70 /180);//70 degrees
  r_foot_spline_.yaw()->addPoint(time, 0);
  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);

    /*
   * To Squat
   */
    time += params_.time_full_squat;
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

    foot_spline_.x()->addPoint(time+0.2, 0);
    foot_spline_.y()->addPoint(time+0.2, params_.foot_distance);
    foot_spline_.z()->addPoint(time+0.2, 0);
    foot_spline_.roll()->addPoint(time+0.2, 0);
    foot_spline_.pitch()->addPoint(time+0.2, 0);
    foot_spline_.yaw()->addPoint(time+0.2, 0);
    r_foot_spline_.x()->addPoint(time+0.2, sin(-M_PI * params_.trunk_overshoot_angle_back /180) * -params_.leg_min_length);
    r_foot_spline_.y()->addPoint(time+0.2, -params_.foot_distance / 2);
    r_foot_spline_.z()->addPoint(time+0.2, cos(-M_PI * params_.trunk_overshoot_angle_back /180) * -params_.leg_min_length);
    r_foot_spline_.roll()->addPoint(time+0.2, 0);
    r_foot_spline_.pitch()->addPoint(time+0.2, -M_PI * params_.trunk_overshoot_angle_back /180);
    r_foot_spline_.yaw()->addPoint(time+0.2, 0);

  calcSquatSplines(time);
}

void DynupEngine::calcSquatSplines(double time) {

  // all positions relative to right foot
  // foot_trajectories_ are for left foot
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
    r_foot_spline_.pitch()->addPoint(time, M_PI * -params_.trunk_pitch /180);
    r_foot_spline_.yaw()->addPoint(time, 0);

    l_hand_spline_.x()->addPoint(time, 0);
    l_hand_spline_.y()->addPoint(time, arm_offset_y_);
    l_hand_spline_.z()->addPoint(time, arm_offset_z_ - arm_max_length_);
    l_hand_spline_.roll()->addPoint(time, 0);
    l_hand_spline_.pitch()->addPoint(time, M_PI/2);
    l_hand_spline_.yaw()->addPoint(time, 0);
    r_hand_spline_.x()->addPoint(time, 0);
    r_hand_spline_.y()->addPoint(time, -arm_offset_y_);
    r_hand_spline_.z()->addPoint(time, arm_offset_z_ - arm_max_length_);
    r_hand_spline_.roll()->addPoint(time, 0);
    r_hand_spline_.pitch()->addPoint(time, M_PI/2);
    r_hand_spline_.yaw()->addPoint(time, 0);

  time += params_.rise_time;
  foot_spline_.x()->addPoint(time, 0);
  foot_spline_.y()->addPoint(time, params_.foot_distance);
  foot_spline_.z()->addPoint(time, 0);
  foot_spline_.roll()->addPoint(time, 0);
  foot_spline_.pitch()->addPoint(time, 0);
  foot_spline_.yaw()->addPoint(time, 0);

  r_foot_spline_.x()->addPoint(time, params_.trunk_x);//TODO: Calculate this from trunk_pitch
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2.0);
  r_foot_spline_.z()->addPoint(time, -params_.trunk_height);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, M_PI * -params_.trunk_pitch /180);
  r_foot_spline_.yaw()->addPoint(time, 0);

    l_hand_spline_.x()->addPoint(time, 0);
    l_hand_spline_.y()->addPoint(time, arm_offset_y_);
    l_hand_spline_.z()->addPoint(time, arm_offset_z_ - arm_max_length_);
    l_hand_spline_.roll()->addPoint(time, 0);
    l_hand_spline_.pitch()->addPoint(time, M_PI/2);
    l_hand_spline_.yaw()->addPoint(time, 0);
    r_hand_spline_.x()->addPoint(time, 0);
    r_hand_spline_.y()->addPoint(time, -arm_offset_y_);
    r_hand_spline_.z()->addPoint(time, arm_offset_z_ - arm_max_length_);
    r_hand_spline_.roll()->addPoint(time, 0);
    r_hand_spline_.pitch()->addPoint(time, M_PI/2);
    r_hand_spline_.yaw()->addPoint(time, 0);
}

void DynupEngine::setGoals(const DynupRequest &goals) {
   initializeSplines(goals.l_hand_pose, goals.r_hand_pose, goals.l_foot_pose, goals.r_foot_pose);
    if (goals.direction == "front") {
        duration_ = params_.time_hands_side +
                    params_.time_hands_front +
                    params_.time_foot_close +
                    params_.time_foot_ground +
                    params_.time_torso_45 +
                    params_.time_to_squat +
                    params_.wait_in_squat +
                    params_.rise_time;
      direction_ = 1;
     calcFrontSplines();
  }
  else if(goals.direction == "back"){
     duration_ = params_.time_legs_close +
                 params_.time_foot_ground +
                 params_.time_squat_push +
                 params_.time_full_squat +
                 params_.wait_in_squat +
                 params_.rise_time;
      direction_ = 0;
     calcBackSplines();
  }
  else {
      duration_ = params_.wait_in_squat +
                  params_.rise_time;
      direction_ = 2;
      calcSquatSplines(0);
  }
}

int DynupEngine::getPercentDone() const {
  return int(time_ / duration_ * 100);
}

/*Calculates if we are at a point of the animation where stabilizing should be applied. */ //TODO: make this nice
bool DynupEngine::isStabilizingNeeded() const {
    return (direction_ == 1 && time_ >= params_.time_hands_side + params_.time_foot_close + params_.time_hands_front +
                                        params_.time_foot_ground + params_.time_torso_45 + params_.time_to_squat) ||
           (direction_ == 0 && time_ >= params_.time_legs_close +
                                        params_.time_hands_down +
                                        params_.time_squat_push +
                                        params_.time_full_squat) ||
            (direction_ == 2);
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
  // update this value
  arm_offset_y_ = shoulder_offset_y_ + params_.arm_side_offset;
}

}
