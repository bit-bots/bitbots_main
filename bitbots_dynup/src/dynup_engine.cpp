#include "bitbots_dynup/dynup_engine.h"
#include <ros/ros.h>
#include <ros/console.h>

namespace bitbots_dynup {

DynupEngine::DynupEngine() : listener_(tf_buffer_) {}

void DynupEngine::init(double arm_max_length, double arm_offset_y, double arm_offset_z) {
  arm_max_length_ = arm_max_length;
  //this are just the offsets to the shoulder, we need to apply some additional offset to prevent collisions
  shoulder_offset_y_ = arm_offset_y;
  arm_offset_z_ = arm_offset_z;
  // call this to initialize some values
  setParams(params_);

  ros::NodeHandle nh;
  pub_engine_debug_ = nh.advertise<bitbots_dynup::DynupEngineDebug>("dynup_engine_debug", 1);
  pub_debug_marker_ = nh.advertise<visualization_msgs::Marker>("dynup_engine_marker", 1);
  marker_id_ = 1;

}

void DynupEngine::reset() {
  time_ = 0;
  duration_ = 0;
  r_foot_spline_ = bitbots_splines::PoseSpline();
  l_hand_spline_ = bitbots_splines::PoseSpline();
  r_hand_spline_ = bitbots_splines::PoseSpline();
  l_foot_spline_ = bitbots_splines::PoseSpline();
}

void DynupEngine::publishDebug() {
    if (pub_engine_debug_.getNumSubscribers() == 0 && pub_debug_marker_.getNumSubscribers() == 0) {
        return;
    }

    bitbots_dynup::DynupEngineDebug msg;
    msg.header.stamp = ros::Time::now();

    msg.time = time_;
    if (direction_ == 1){
      if(time_ < params_.time_hands_side){
          msg.state_number = 0;
      }else if(time_ < params_.time_hands_side + params_.time_hands_rotate){
          msg.state_number = 1;
      }else if(time_ < params_.time_hands_side + params_.time_hands_rotate + params_.time_foot_close){
          msg.state_number = 2;
      }else if(time_ < params_.time_hands_side + params_.time_hands_rotate + params_.time_foot_close + params_.time_hands_front){
          msg.state_number = 3;
      }else if(time_ < params_.time_hands_side + params_.time_hands_rotate + params_.time_foot_close + params_.time_hands_front + params_.time_foot_ground_front){
          msg.state_number = 4;
      }else if(time_ < params_.time_hands_side + params_.time_hands_rotate + params_.time_foot_close + params_.time_hands_front + params_.time_foot_ground_front + params_.time_torso_45){
          msg.state_number = 5;
      }else if(time_ < params_.time_hands_side + params_.time_hands_rotate + params_.time_foot_close + params_.time_hands_front + params_.time_foot_ground_front + params_.time_torso_45 + params_.time_to_squat){
          msg.state_number = 6;
      }else {
          msg.state_number = 7;
      }
    }else if (direction_ == 0) {
      if(time_ < params_.time_legs_close){
          msg.state_number = 0;
      }else if(time_ < params_.time_legs_close + params_.time_foot_ground_back){
          msg.state_number = 1;
      }else if(time_ < params_.time_legs_close + params_.time_foot_ground_back + params_.time_squat_push){
          msg.state_number = 2;
      }else if(time_ < params_.time_legs_close + params_.time_foot_ground_back + params_.time_squat_push + params_.time_full_squat){
          msg.state_number = 3;
      }else{
          msg.state_number = 4;
      }
    }else{
          msg.state_number = -1;
    }

    geometry_msgs::Pose l_arm_pose;
    tf2::toMsg(goals_.l_hand_goal_pose, l_arm_pose);
    msg.l_arm_pose = l_arm_pose;
    publishArrowMarker("l_arm", "base_link", l_arm_pose, 1, 0, 0, 1);

    geometry_msgs::Pose l_arm_from_shoulder;
    tf2::toMsg(l_hand_spline_.getTfTransform(time_), l_arm_from_shoulder);
    msg.l_arm_pose_from_shoulder = l_arm_from_shoulder;

    geometry_msgs::Pose r_arm_pose;
    tf2::toMsg(goals_.r_hand_goal_pose, r_arm_pose);
    msg.r_arm_pose = r_arm_pose;
    publishArrowMarker("r_arm", "base_link", r_arm_pose, 0, 1, 0, 1);

    geometry_msgs::Pose r_arm_from_shoulder;
    tf2::toMsg(r_hand_spline_.getTfTransform(time_), r_arm_from_shoulder);
    msg.r_arm_pose_from_shoulder = r_arm_from_shoulder;

    geometry_msgs::Pose l_leg_pose;
    tf2::toMsg(goals_.l_foot_goal_pose, l_leg_pose);
    msg.l_leg_pose = l_leg_pose;

    geometry_msgs::Pose l_leg_pose_in_base_link;
    tf2::toMsg(goals_.r_foot_goal_pose * goals_.l_foot_goal_pose, l_leg_pose_in_base_link);
    publishArrowMarker("l_leg_pose", "base_link", l_leg_pose_in_base_link, 0, 0, 1, 1);

    geometry_msgs::Pose r_leg_pose;
    tf2::toMsg(goals_.r_foot_goal_pose, r_leg_pose);
    msg.r_leg_pose = r_leg_pose;
    publishArrowMarker("r_leg_pose", "base_link", r_leg_pose, 0, 1, 1, 1);

    double r,p,y;
    tf2::Quaternion q;
    tf2::convert(r_leg_pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(r, p, y);
    msg.foot_roll = r;
    msg.foot_pitch = p;
    msg.foot_yaw = y;

    pub_engine_debug_.publish(msg);
}

//TODO this method does also exist in walking. should maybe be refactored and put into splines
//TODO generally the visualization should maybe be put into the visualizer class
void DynupEngine::publishArrowMarker(std::string name_space,
                                        std::string frame,
                                        geometry_msgs::Pose pose, float r, float g, float b, float a) {
  visualization_msgs::Marker marker_msg;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.header.frame_id = frame;

  marker_msg.type = marker_msg.ARROW;
  marker_msg.ns = name_space;
  marker_msg.action = marker_msg.ADD;
  marker_msg.pose = pose;

  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  marker_msg.color = color;

  geometry_msgs::Vector3 scale;
  scale.x = 0.01;
  scale.y = 0.003;
  scale.z = 0.003;
  marker_msg.scale = scale;

  marker_msg.id = marker_id_;
  marker_id_++;

  pub_debug_marker_.publish(marker_msg);
}

DynupResponse DynupEngine::update(double dt) {
  // TODO what happens when splines for foot and trunk are not present?

  /* Get should-be pose from planned splines (every axis) at current time */
  tf2::Transform l_foot_pose = l_foot_spline_.getTfTransform(time_);
  tf2::Transform r_foot_pose = r_foot_spline_.getTfTransform(time_);
  tf2::Transform l_hand_pose = l_hand_spline_.getTfTransform(time_);
  tf2::Transform r_hand_pose = r_hand_spline_.getTfTransform(time_);

  // add offsets to transform to base_link
  goals_.l_foot_goal_pose = l_foot_pose;
  goals_.r_foot_goal_pose = r_foot_pose;
  goals_.l_hand_goal_pose = offset_left_ * l_hand_pose;
  goals_.r_hand_goal_pose = offset_right_ * r_hand_pose;
  goals_.is_stabilizing_needed = isStabilizingNeeded();

  publishDebug();

  time_ += dt;

  return goals_;
}

void DynupEngine::initializeSpline(geometry_msgs::Pose pose, bitbots_splines::PoseSpline spline) {
    double r,p,y;
    tf2::Quaternion q;

    spline.x()->addPoint(0.0, pose.position.x);
    // substract offsets to switch the frame
    spline.y()->addPoint(0.0, pose.position.y);
    spline.z()->addPoint(0.0, pose.position.z);
    tf2::convert(pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(r, p, y);
    spline.roll()->addPoint(0.0, r);
    spline.pitch()->addPoint(0.0, p);
    spline.yaw()->addPoint(0.0, y);
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
  l_hand_spline_.y()->addPoint(time, arm_max_length_);
  l_hand_spline_.z()->addPoint(time, 0);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, M_PI/2);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -arm_max_length_);
  r_hand_spline_.z()->addPoint(time, 0);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, -M_PI/2);

  /*
   * rotate hands
   */
  time += params_.time_hands_rotate;
  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, arm_max_length_);
  l_hand_spline_.z()->addPoint(time, 0);
  l_hand_spline_.roll()->addPoint(time, -M_PI/2);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, M_PI/2);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -arm_max_length_);
  r_hand_spline_.z()->addPoint(time, 0);
  r_hand_spline_.roll()->addPoint(time, M_PI/2);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, -M_PI/2);

  /*
   * pull legs to body
   */
  // compute foot position while standing up
  double foot_x = sin(M_PI * -params_.max_leg_angle /180) * -params_.leg_min_length;
  double foot_z = cos(M_PI * -params_.max_leg_angle /180) * -params_.leg_min_length;
  double foot_pitch = -M_PI * params_.max_leg_angle /180;
  time += params_.time_foot_close;
  r_foot_spline_.x()->addPoint(time, foot_x -params_.trunk_x);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, foot_z);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, foot_pitch);
  r_foot_spline_.yaw()->addPoint(time, 0);
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);


  /*
   * hands to the front
   */
  time += params_.time_hands_front;
  // we apply a small offset in x direction, to make sure that ShoulderPitch does not go > 180° due to rounding errors
  double x_offset = 0.01;
  // hacky extra spline point since we need to accelerate quickly
  r_hand_spline_.x()->addPoint(time - 0.01, x_offset);
  r_hand_spline_.x()->addPoint(time, x_offset, 1);
  r_hand_spline_.y()->addPoint(time, 0);
  r_hand_spline_.z()->addPoint(time, arm_max_length_);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, -M_PI/2);
  r_hand_spline_.yaw()->addPoint(time, 0);
  l_hand_spline_.x()->addPoint(time - 0.01, x_offset);
  l_hand_spline_.x()->addPoint(time, x_offset, 1);
  l_hand_spline_.y()->addPoint(time, 0);
  l_hand_spline_.z()->addPoint(time, arm_max_length_);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, -M_PI/2);
  l_hand_spline_.yaw()->addPoint(time, 0);



  /*
   * Foot under body
   */
  time += params_.time_foot_ground_front;
  r_foot_spline_.x()->addPoint(time, foot_x -params_.trunk_x);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, foot_z);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, foot_pitch);
  r_foot_spline_.yaw()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);


  /*
   * Torso 45°
   */
  time += params_.time_torso_45;
  r_hand_spline_.x()->addPoint(time, arm_max_length_);
  r_hand_spline_.y()->addPoint(time, 0);
  r_hand_spline_.z()->addPoint(time, 0);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, 0);
  l_hand_spline_.x()->addPoint(time, arm_max_length_);
  l_hand_spline_.y()->addPoint(time, 0);
  l_hand_spline_.z()->addPoint(time, 0);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, 0);

  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);
  r_foot_spline_.x()->addPoint(time, foot_x -params_.trunk_x);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, foot_z);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, foot_pitch);
  r_foot_spline_.yaw()->addPoint(time, 0);

  /*
   * To Squat
   */
  time += params_.time_to_squat;
  r_hand_spline_.x()->addPoint(time, 0, -1);
  r_hand_spline_.x()->addPoint(time + 0.01, 0);
  r_hand_spline_.y()->addPoint(time, 0);
  r_hand_spline_.z()->addPoint(time, -arm_max_length_);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI/2);
  r_hand_spline_.yaw()->addPoint(time, 0);
  l_hand_spline_.x()->addPoint(time, 0, -1);
  l_hand_spline_.x()->addPoint(time + 0.01, 0);
  l_hand_spline_.y()->addPoint(time, 0);
  l_hand_spline_.z()->addPoint(time, -arm_max_length_);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI/2);
  l_hand_spline_.yaw()->addPoint(time, 0);

  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);
  r_foot_spline_.x()->addPoint(time, foot_x -params_.trunk_x);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, foot_z);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, foot_pitch);
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
  r_foot_spline_.x()->addPoint(time, -params_.trunk_x);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, 0);
  r_foot_spline_.yaw()->addPoint(time, 0);
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);

    l_hand_spline_.x()->addPoint(time, -0.15);
    l_hand_spline_.y()->addPoint(time, 0);
    l_hand_spline_.z()->addPoint(time, -0.15);//TODO: Hacky hack, not actual position
    l_hand_spline_.roll()->addPoint(time, 0);
    l_hand_spline_.pitch()->addPoint(time, M_PI/2);
    l_hand_spline_.yaw()->addPoint(time, 0);
    r_hand_spline_.x()->addPoint(time, -0.15);
    r_hand_spline_.y()->addPoint(time, 0);
    r_hand_spline_.z()->addPoint(time, -0.1);
    r_hand_spline_.roll()->addPoint(time, 0);
    r_hand_spline_.pitch()->addPoint(time, M_PI/2);
    r_hand_spline_.yaw()->addPoint(time, 0);

  /*
   * Foot under body
   */
  time += params_.time_foot_ground_back;
    l_hand_spline_.x()->addPoint(time, -sqrt(2* pow(arm_max_length_/2, 2)));
    l_hand_spline_.y()->addPoint(time, 0);
    l_hand_spline_.z()->addPoint(time, 0.28); //TODO: Also hacky hack, forcing elbow angle
    l_hand_spline_.roll()->addPoint(time, 0);
    l_hand_spline_.pitch()->addPoint(time, M_PI*0.6);
    l_hand_spline_.yaw()->addPoint(time, 0);
    r_hand_spline_.x()->addPoint(time, -sqrt(2* pow(arm_max_length_/2, 2)));
    r_hand_spline_.y()->addPoint(time, 0);
    r_hand_spline_.z()->addPoint(time, 0.28);
    r_hand_spline_.roll()->addPoint(time, 0);
    r_hand_spline_.pitch()->addPoint(time, M_PI*0.6);
    r_hand_spline_.yaw()->addPoint(time, 0);

  r_foot_spline_.x()->addPoint(time, -sin(1.22173) * params_.leg_min_length -params_.trunk_x);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -cos(1.22173) * params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, M_PI * 70 /180);//70 degrees
  r_foot_spline_.yaw()->addPoint(time, 0);
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);

  /*
   * Halfway
   */
  time += params_.time_squat_push;
  l_hand_spline_.x()->addPoint(time, -sqrt(2* pow(arm_max_length_/2, 2)));
  l_hand_spline_.y()->addPoint(time, 0);
  l_hand_spline_.z()->addPoint(time, 0.28); //TODO: Also hacky hack, forcing elbow angle
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI*0.9);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, -sqrt(2* pow(arm_max_length_/2, 2)));
  r_hand_spline_.y()->addPoint(time, 0);
  r_hand_spline_.z()->addPoint(time, 0.28);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI*0.9);
  r_hand_spline_.yaw()->addPoint(time, 0);


  r_foot_spline_.x()->addPoint(time, -sin(0.872665) * params_.leg_min_length -params_.trunk_x);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -cos(0.872665) * params_.leg_min_length);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, M_PI * 70 /180);//70 degrees
  r_foot_spline_.yaw()->addPoint(time, 0);
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);

    /*
   * To Squat
   */
    time += params_.time_full_squat;
    l_hand_spline_.x()->addPoint(time, -arm_max_length_);
    l_hand_spline_.y()->addPoint(time, 0);
    l_hand_spline_.z()->addPoint(time, 0);
    l_hand_spline_.roll()->addPoint(time, 0);
    l_hand_spline_.pitch()->addPoint(time, M_PI);
    l_hand_spline_.yaw()->addPoint(time, 0);
    r_hand_spline_.x()->addPoint(time, -arm_max_length_);
    r_hand_spline_.y()->addPoint(time, 0);
    r_hand_spline_.z()->addPoint(time, 0);
    r_hand_spline_.roll()->addPoint(time, 0);
    r_hand_spline_.pitch()->addPoint(time, M_PI);
    r_hand_spline_.yaw()->addPoint(time, 0);

    l_foot_spline_.x()->addPoint(time + 0.2, 0);
    l_foot_spline_.y()->addPoint(time + 0.2, params_.foot_distance);
    l_foot_spline_.z()->addPoint(time + 0.2, 0);
    l_foot_spline_.roll()->addPoint(time + 0.2, 0);
    l_foot_spline_.pitch()->addPoint(time + 0.2, 0);
    l_foot_spline_.yaw()->addPoint(time + 0.2, 0);
    r_foot_spline_.x()->addPoint(time+0.2, sin(-M_PI * params_.trunk_overshoot_angle_back /180) * -params_.leg_min_length -params_.trunk_x);
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
    l_foot_spline_.x()->addPoint(time, 0);
    l_foot_spline_.y()->addPoint(time, params_.foot_distance);
    l_foot_spline_.z()->addPoint(time, 0);
    l_foot_spline_.roll()->addPoint(time, 0);
    l_foot_spline_.pitch()->addPoint(time, 0);
    l_foot_spline_.yaw()->addPoint(time, 0);
    r_foot_spline_.x()->addPoint(time, -params_.trunk_x);
    r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2);
    r_foot_spline_.z()->addPoint(time, -params_.leg_min_length);
    r_foot_spline_.roll()->addPoint(time, 0);
    r_foot_spline_.pitch()->addPoint(time, M_PI * -params_.trunk_pitch /180);
    r_foot_spline_.yaw()->addPoint(time, 0);

    l_hand_spline_.x()->addPoint(time, 0);
    l_hand_spline_.y()->addPoint(time, 0);
    l_hand_spline_.z()->addPoint(time, -arm_max_length_);
    l_hand_spline_.roll()->addPoint(time, 0);
    l_hand_spline_.pitch()->addPoint(time, M_PI/2);
    l_hand_spline_.yaw()->addPoint(time, 0);
    r_hand_spline_.x()->addPoint(time, 0);
    r_hand_spline_.y()->addPoint(time, 0);
    r_hand_spline_.z()->addPoint(time, -arm_max_length_);
    r_hand_spline_.roll()->addPoint(time, 0);
    r_hand_spline_.pitch()->addPoint(time, M_PI/2);
    r_hand_spline_.yaw()->addPoint(time, 0);

  time += params_.rise_time;
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);

  r_foot_spline_.x()->addPoint(time, -params_.trunk_x);
  r_foot_spline_.y()->addPoint(time, -params_.foot_distance / 2.0);
  r_foot_spline_.z()->addPoint(time, -params_.trunk_height);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, M_PI * -params_.trunk_pitch /180);
  r_foot_spline_.yaw()->addPoint(time, 0);

    l_hand_spline_.x()->addPoint(time, 0);
    l_hand_spline_.y()->addPoint(time, 0);
    l_hand_spline_.z()->addPoint(time, -arm_max_length_);
    l_hand_spline_.roll()->addPoint(time, 0);
    l_hand_spline_.pitch()->addPoint(time, M_PI/2);
    l_hand_spline_.yaw()->addPoint(time, 0);
    r_hand_spline_.x()->addPoint(time, 0);
    r_hand_spline_.y()->addPoint(time, 0);
    r_hand_spline_.z()->addPoint(time, -arm_max_length_);
    r_hand_spline_.roll()->addPoint(time, 0);
    r_hand_spline_.pitch()->addPoint(time, M_PI/2);
    r_hand_spline_.yaw()->addPoint(time, 0);

    //TODO move hands back into a walkready pose
}

void DynupEngine::setGoals(const DynupRequest &goals) {
    initializeSpline(goals.l_hand_pose, l_hand_spline_);
    initializeSpline(goals.r_hand_pose, r_hand_spline_);
    initializeSpline(goals.l_foot_pose, l_foot_spline_);
    initializeSpline(goals.r_foot_pose, r_foot_spline_);
    if (goals.direction == "front") {
        duration_ = params_.time_hands_side +
                    params_.time_hands_rotate +
                    params_.time_hands_front +
                    params_.time_foot_close +
                    params_.time_foot_ground_front +
                    params_.time_torso_45 +
                    params_.time_to_squat +
                    params_.wait_in_squat +
                    params_.rise_time;
      direction_ = 1;
     calcFrontSplines();
  }
  else if(goals.direction == "back"){
     duration_ = params_.time_legs_close +
                 params_.time_foot_ground_back +
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
                                        params_.time_foot_ground_front + params_.time_torso_45 + params_.time_to_squat) ||
           (direction_ == 0 && time_ >= params_.time_legs_close +
                                        params_.time_squat_push +
                                        params_.time_full_squat) ||
            (direction_ == 2);
}

bitbots_splines::PoseSpline DynupEngine::getRFootSplines() const {
  return r_foot_spline_;
}

bitbots_splines::PoseSpline DynupEngine::getLFootSplines() const {
    return l_foot_spline_;
}

bitbots_splines::PoseSpline DynupEngine::getRHandSplines() const {
    return r_hand_spline_;
}

bitbots_splines::PoseSpline DynupEngine::getLHandSplines() const {
    return l_hand_spline_;
}

void DynupEngine::setParams(DynUpConfig params) {
  params_ = params;
  // update this values
  arm_offset_y_ = shoulder_offset_y_ + params_.arm_side_offset;
  offset_left_ = tf2::Transform(tf2::Quaternion(0,0,0,1), {0, arm_offset_y_, arm_offset_z_});
  offset_right_ = tf2::Transform(tf2::Quaternion(0,0,0,1), {0, -arm_offset_y_, arm_offset_z_});
}

}
