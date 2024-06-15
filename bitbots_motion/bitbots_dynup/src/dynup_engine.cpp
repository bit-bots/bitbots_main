#include "bitbots_dynup/dynup_engine.hpp"

#include <rclcpp/rclcpp.hpp>

namespace bitbots_dynup {

DynupEngine::DynupEngine(rclcpp::Node::SharedPtr node) : node_(node) {
  pub_engine_debug_ = node_->create_publisher<bitbots_dynup::msg::DynupEngineDebug>("dynup_engine_debug", 1);
  pub_debug_marker_ = node_->create_publisher<visualization_msgs::msg::Marker>("dynup_engine_marker", 1);
  // We need a separate node for the parameter client because the parameter client adds the node to an executor
  // and our dynup node is already added to an executor
  walking_param_node_ = std::make_shared<rclcpp::Node>(std::string(node->get_name()) + "_walking_param_node");
  walking_param_client_ = std::make_shared<rclcpp::SyncParametersClient>(walking_param_node_, "/walking");
}

void DynupEngine::init(double arm_offset_y, double arm_offset_z) {
  // this are just the offsets to the shoulder, we need to apply some additional offset to prevent collisions
  shoulder_offset_y_ = arm_offset_y;
  arm_offset_z_ = arm_offset_z;
  setParams(params_);

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

void DynupEngine::reset(double time) {
  time_ = time;
  duration_ = 0;
  r_foot_spline_ = bitbots_splines::PoseSpline();
  l_hand_spline_ = bitbots_splines::PoseSpline();
  r_hand_spline_ = bitbots_splines::PoseSpline();
  l_foot_spline_ = bitbots_splines::PoseSpline();
}

void DynupEngine::publishDebug() {
  if (pub_engine_debug_->get_subscription_count() == 0 && pub_debug_marker_->get_subscription_count() == 0) {
    return;
  }

  bitbots_dynup::msg::DynupEngineDebug msg;
  msg.header.stamp = node_->get_clock()->now();

  msg.time = time_;
  msg.stabilization_active = isStabilizingNeeded();
  if (direction_ == 1) {
    if (time_ < params_.dynup_front.time_hands_side) {
      msg.state_number = 0;
    } else if (time_ < params_.dynup_front.time_hands_side + params_.dynup_front.time_hands_rotate) {
      msg.state_number = 1;
    } else if (time_ < params_.dynup_front.time_hands_side + params_.dynup_front.time_hands_rotate +
                           params_.dynup_front.time_hands_front) {
      msg.state_number = 2;
    } else if (time_ < params_.dynup_front.time_hands_side + params_.dynup_front.time_hands_rotate +
                           params_.dynup_front.time_hands_front + params_.dynup_front.time_foot_close) {
      msg.state_number = 3;
    } else if (time_ < params_.dynup_front.time_hands_side + params_.dynup_front.time_hands_rotate +
                           params_.dynup_front.time_hands_front + params_.dynup_front.time_foot_close +
                           params_.dynup_front.time_foot_ground_front) {
      msg.state_number = 4;
    } else if (time_ < params_.dynup_front.time_hands_side + params_.dynup_front.time_hands_rotate +
                           params_.dynup_front.time_hands_front + params_.dynup_front.time_foot_close +
                           params_.dynup_front.time_foot_ground_front + params_.dynup_front.time_torso_45) {
      msg.state_number = 5;
    } else if (time_ < params_.dynup_front.time_hands_side + params_.dynup_front.time_hands_rotate +
                           params_.dynup_front.time_hands_front + params_.dynup_front.time_foot_close +
                           params_.dynup_front.time_foot_ground_front + params_.dynup_front.time_torso_45 +
                           params_.dynup_front.time_to_squat) {
      msg.state_number = 6;
    } else if (time_ < params_.dynup_front.time_hands_side + params_.dynup_front.time_hands_rotate +
                           params_.dynup_front.time_hands_front + params_.dynup_front.time_foot_close +
                           params_.dynup_front.time_foot_ground_front + params_.dynup_front.time_torso_45 +
                           params_.dynup_front.time_to_squat + params_.dynup_front.wait_in_squat_front) {
      msg.state_number = 7;
    } else {
      msg.state_number = 8;
    }
  } else if (direction_ == 0) {
    if (time_ < params_.dynup_back.time_legs_close) {
      msg.state_number = 0;
    } else if (time_ < params_.dynup_back.time_legs_close + params_.dynup_back.time_foot_ground_back) {
      msg.state_number = 1;
    } else if (time_ < params_.dynup_back.time_legs_close + params_.dynup_back.time_foot_ground_back +
                           params_.dynup_back.time_full_squat_hands) {
      msg.state_number = 2;
    } else if (time_ < params_.dynup_back.time_legs_close + params_.dynup_back.time_foot_ground_back +
                           params_.dynup_back.time_full_squat_hands + params_.dynup_back.time_full_squat_legs) {
      msg.state_number = 3;
    } else if (time_ < params_.dynup_back.time_legs_close + params_.dynup_back.time_foot_ground_back +
                           params_.dynup_back.time_full_squat_hands + params_.dynup_back.time_full_squat_legs +
                           params_.dynup_back.wait_in_squat_back) {
      msg.state_number = 4;
    } else {
      msg.state_number = 5;
    }
  } else {
    msg.state_number = -1;
  }

  geometry_msgs::msg::Pose l_arm_pose;
  tf2::toMsg(goals_.l_hand_goal_pose, l_arm_pose);
  msg.l_arm_pose = l_arm_pose;
  publishArrowMarker("l_arm", "base_link", l_arm_pose, 1, 0, 0, 1);

  geometry_msgs::msg::Pose l_arm_from_shoulder;
  tf2::toMsg(l_hand_spline_.getTfTransform(time_), l_arm_from_shoulder);
  msg.l_arm_pose_from_shoulder = l_arm_from_shoulder;

  geometry_msgs::msg::Pose r_arm_pose;
  tf2::toMsg(goals_.r_hand_goal_pose, r_arm_pose);
  msg.r_arm_pose = r_arm_pose;
  publishArrowMarker("r_arm", "base_link", r_arm_pose, 0, 1, 0, 1);

  geometry_msgs::msg::Pose r_arm_from_shoulder;
  tf2::toMsg(r_hand_spline_.getTfTransform(time_), r_arm_from_shoulder);
  msg.r_arm_pose_from_shoulder = r_arm_from_shoulder;

  geometry_msgs::msg::Pose l_leg_pose;
  tf2::toMsg(goals_.l_foot_goal_pose, l_leg_pose);
  msg.l_leg_pose = l_leg_pose;

  geometry_msgs::msg::Pose l_leg_pose_in_base_link;
  tf2::toMsg(goals_.r_foot_goal_pose * goals_.l_foot_goal_pose, l_leg_pose_in_base_link);
  publishArrowMarker("l_leg_pose", "base_link", l_leg_pose_in_base_link, 0, 0, 1, 1);

  geometry_msgs::msg::Pose r_leg_pose;
  tf2::toMsg(goals_.r_foot_goal_pose, r_leg_pose);
  msg.r_leg_pose = r_leg_pose;
  publishArrowMarker("r_leg_pose", "base_link", r_leg_pose, 0, 1, 1, 1);

  double r, p, y;
  tf2::Quaternion q;
  tf2::convert(r_leg_pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  msg.foot_roll = r;
  msg.foot_pitch = p;
  msg.foot_yaw = y;

  pub_engine_debug_->publish(msg);
}

void DynupEngine::publishArrowMarker(std::string name_space, std::string frame, geometry_msgs::msg::Pose pose, float r,
                                     float g, float b, float a) {
  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header.stamp = node_->get_clock()->now();
  marker_msg.header.frame_id = frame;

  marker_msg.type = marker_msg.ARROW;
  marker_msg.ns = name_space;
  marker_msg.action = marker_msg.ADD;
  marker_msg.pose = pose;

  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  marker_msg.color = color;

  geometry_msgs::msg::Vector3 scale;
  scale.x = 0.01;
  scale.y = 0.003;
  scale.z = 0.003;
  marker_msg.scale = scale;

  marker_msg.id = marker_id_;
  marker_id_++;

  pub_debug_marker_->publish(marker_msg);
}

DynupResponse DynupEngine::update(double dt) {
  time_ = std::min(time_, duration_);
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
  goals_.is_head_zero = isHeadZero();

  publishDebug();

  time_ += dt;

  return goals_;
}

bitbots_splines::PoseSpline DynupEngine::initializeSpline(geometry_msgs::msg::Pose pose,
                                                          bitbots_splines::PoseSpline spline) {
  double r, p, y;
  tf2::Quaternion q;

  spline.x()->addPoint(0.0, pose.position.x);
  spline.y()->addPoint(0.0, pose.position.y);
  spline.z()->addPoint(0.0, pose.position.z);
  tf2::convert(pose.orientation, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
  spline.roll()->addPoint(0.0, r);
  spline.pitch()->addPoint(0.0, p);
  spline.yaw()->addPoint(0.0, y);

  return spline;
}

double DynupEngine::calcFrontSplines() {
  /*
  calculates splines for front up
  */

  /*
   * Pose 0: Extend hands to the side, for later turning
   */
  double time = params_.dynup_front.time_hands_side;
  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, params_.end_pose.arm_extended_length);
  l_hand_spline_.z()->addPoint(time, 0);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, M_PI / 2);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -params_.end_pose.arm_extended_length);
  r_hand_spline_.z()->addPoint(time, 0);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, -M_PI / 2);

  /*
   * Pose 1: Rotate hands to the orientation that we will need later
   */
  double pitch_offset = params_.dynup_front.hands_pitch / 180 * M_PI;
  time += params_.dynup_front.time_hands_rotate;
  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, params_.end_pose.arm_extended_length);
  l_hand_spline_.z()->addPoint(time, 0);
  l_hand_spline_.roll()->addPoint(time, -M_PI / 2);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, M_PI / 2);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -params_.end_pose.arm_extended_length);
  r_hand_spline_.z()->addPoint(time, 0);
  r_hand_spline_.roll()->addPoint(time, M_PI / 2);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, -M_PI / 2);

  /*
   * Pose 2: Move hands to the front
   */
  time += params_.dynup_front.time_hands_front;
  // we apply a small offset in x direction, to make sure that ShoulderPitch does not go > 180° due to rounding errors
  double x_offset = 0.005;
  r_hand_spline_.x()->addPoint(time, sin(pitch_offset) * x_offset);
  r_hand_spline_.y()->addPoint(time, -params_.dynup_front.arm_side_offset_front);
  r_hand_spline_.z()->addPoint(time, cos(pitch_offset) * params_.end_pose.arm_extended_length);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, -M_PI / 2 + pitch_offset);
  r_hand_spline_.yaw()->addPoint(time, 0);
  l_hand_spline_.x()->addPoint(time, sin(pitch_offset) * x_offset);
  l_hand_spline_.y()->addPoint(time, params_.dynup_front.arm_side_offset_front);
  l_hand_spline_.z()->addPoint(time, cos(pitch_offset) * params_.end_pose.arm_extended_length);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, -M_PI / 2 + pitch_offset);
  l_hand_spline_.yaw()->addPoint(time, 0);

  // keep feet on initial pose till this point, otherwise robot will tilt and arms will collide with floor
  r_foot_spline_.x()->addPoint(time, r_foot_spline_.x()->points()[0].position);
  r_foot_spline_.y()->addPoint(time, r_foot_spline_.y()->points()[0].position);
  r_foot_spline_.z()->addPoint(time, r_foot_spline_.z()->points()[0].position);
  r_foot_spline_.roll()->addPoint(time, r_foot_spline_.roll()->points()[0].position);
  r_foot_spline_.pitch()->addPoint(time, r_foot_spline_.pitch()->points()[0].position);
  r_foot_spline_.yaw()->addPoint(time, r_foot_spline_.yaw()->points()[0].position);
  l_foot_spline_.x()->addPoint(time, l_foot_spline_.x()->points()[0].position);
  l_foot_spline_.y()->addPoint(time, l_foot_spline_.y()->points()[0].position);
  l_foot_spline_.z()->addPoint(time, l_foot_spline_.z()->points()[0].position);
  l_foot_spline_.roll()->addPoint(time, l_foot_spline_.roll()->points()[0].position);
  l_foot_spline_.pitch()->addPoint(time, l_foot_spline_.pitch()->points()[0].position);
  l_foot_spline_.yaw()->addPoint(time, l_foot_spline_.yaw()->points()[0].position);

  /*
   * Pose 3: Pull legs to body
   */
  time += params_.dynup_front.time_foot_close;
  r_foot_spline_.x()->addPoint(time, -params_.dynup_front.trunk_x_front);
  r_foot_spline_.y()->addPoint(time, -params_.end_pose.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -params_.dynup_front.leg_min_length_front);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, 0);
  r_foot_spline_.yaw()->addPoint(time, 0);
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.end_pose.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);

  /*
   * Pose 4: Position feet under body
   */
  // compute foot position while standing up
  double foot_x = sin(M_PI * -params_.dynup_front.max_leg_angle / 180) * -params_.dynup_front.leg_min_length_front;
  double foot_z = cos(M_PI * -params_.dynup_front.max_leg_angle / 180) * -params_.dynup_front.leg_min_length_front;
  double foot_pitch = -M_PI * params_.dynup_front.max_leg_angle / 180;
  time += params_.dynup_front.time_foot_ground_front;
  r_foot_spline_.x()->addPoint(time, foot_x - params_.dynup_front.trunk_x_front);
  r_foot_spline_.y()->addPoint(time, -params_.end_pose.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, foot_z);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, foot_pitch);
  r_foot_spline_.yaw()->addPoint(time, 0);
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.end_pose.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);

  /*
   * Pose 5: Rotate torso to 45°
   */
  time += params_.dynup_front.time_torso_45;
  r_hand_spline_.x()->addPoint(time, params_.end_pose.arm_extended_length);
  r_hand_spline_.y()->addPoint(time, -params_.dynup_front.arm_side_offset_front);
  r_hand_spline_.z()->addPoint(time, 0);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, 0);
  r_hand_spline_.yaw()->addPoint(time, 0);
  l_hand_spline_.x()->addPoint(time, params_.end_pose.arm_extended_length);
  l_hand_spline_.y()->addPoint(time, params_.dynup_front.arm_side_offset_front);
  l_hand_spline_.z()->addPoint(time, 0);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, 0);
  l_hand_spline_.yaw()->addPoint(time, 0);

  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.end_pose.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);
  r_foot_spline_.x()->addPoint(time, foot_x - params_.dynup_front.trunk_x_front);
  r_foot_spline_.y()->addPoint(time, -params_.end_pose.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, foot_z);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, foot_pitch);
  r_foot_spline_.yaw()->addPoint(time, 0);

  /*
   * Pose 6: Move to squat position
   */
  double angle_foot = -M_PI * params_.dynup_front.trunk_overshoot_angle_front / 180;
  time += params_.dynup_front.time_to_squat;
  // r_hand_spline_.x()->addPoint(time, 0, -1);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -params_.dynup_front.arm_side_offset_front);
  r_hand_spline_.z()->addPoint(time, -params_.end_pose.arm_extended_length);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI / 2);
  r_hand_spline_.yaw()->addPoint(time, 0);
  // l_hand_spline_.x()->addPoint(time, 0, -1);
  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, params_.dynup_front.arm_side_offset_front);
  l_hand_spline_.z()->addPoint(time, -params_.end_pose.arm_extended_length);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI / 2);
  l_hand_spline_.yaw()->addPoint(time, 0);

  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.end_pose.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);
  r_foot_spline_.x()->addPoint(time, -cos(angle_foot) * params_.dynup_front.trunk_x_front);
  r_foot_spline_.y()->addPoint(time, -params_.end_pose.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -sin(angle_foot) * params_.dynup_front.trunk_x_front -
                                         cos(angle_foot) * params_.dynup_front.leg_min_length_front);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, angle_foot);
  r_foot_spline_.yaw()->addPoint(time, 0);

  /*
   * Pose 7: Wait in squat to let instabilities settle
   */
  time += params_.dynup_front.wait_in_squat_front;
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.end_pose.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);
  r_foot_spline_.x()->addPoint(time, -cos(angle_foot) * params_.dynup_front.trunk_x_front);
  r_foot_spline_.y()->addPoint(time, -params_.end_pose.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -sin(angle_foot) * params_.dynup_front.trunk_x_front -
                                         cos(angle_foot) * params_.dynup_front.leg_min_length_front);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, angle_foot);
  r_foot_spline_.yaw()->addPoint(time, 0);

  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, params_.dynup_front.arm_side_offset_front);
  l_hand_spline_.z()->addPoint(time, -params_.end_pose.arm_extended_length);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI / 2);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -params_.dynup_front.arm_side_offset_front);
  r_hand_spline_.z()->addPoint(time, -params_.end_pose.arm_extended_length);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI / 2);
  r_hand_spline_.yaw()->addPoint(time, 0);
  return time;
}

double DynupEngine::calcBackSplines() {
  /*
  calculates splines for back up
  */

  /*
   * Pose 0: Pull legs to body and position hands behind back
   */
  double time = params_.dynup_back.time_legs_close;
  r_foot_spline_.x()->addPoint(time, 0);
  // y is always just related to the foot distance parameter
  r_foot_spline_.y()->addPoint(time, -params_.end_pose.foot_distance / 2);
  // pull legs as closely as possible
  r_foot_spline_.z()->addPoint(time, -params_.dynup_back.leg_min_length_back);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, 0);
  r_foot_spline_.yaw()->addPoint(time, 0);
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.end_pose.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);

  // put hands at specified place behind back to push the torso upwards
  l_hand_spline_.x()->addPoint(time, -params_.dynup_back.hands_behind_back_x);
  l_hand_spline_.y()->addPoint(time, params_.end_pose.arm_side_offset_back);
  l_hand_spline_.z()->addPoint(time, -params_.dynup_back.hands_behind_back_z);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI / 2);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, -params_.dynup_back.hands_behind_back_x);
  r_hand_spline_.y()->addPoint(time, -params_.end_pose.arm_side_offset_back);
  r_hand_spline_.z()->addPoint(time, -params_.dynup_back.hands_behind_back_z);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI / 2);
  r_hand_spline_.yaw()->addPoint(time, 0);

  /*
   * Pose 1: Push torso up with an angle and get feet under body. CoM has to get over support polygon
   */
  time += params_.dynup_back.time_foot_ground_back;
  // length(upper arm) == length(lower arm) -> Isosceles triangle with length of one side :=
  // params_.arm_extended_length/2 set elbow to 90°, we can get x by Pythagorean theorem
  l_hand_spline_.x()->addPoint(time, sin(params_.dynup_back.arms_angle_back * M_PI / 180) *
                                         -sqrt(2 * pow(params_.end_pose.arm_extended_length / 2, 2)));
  l_hand_spline_.y()->addPoint(time, 0);
  l_hand_spline_.z()->addPoint(time, cos(params_.dynup_back.arms_angle_back * M_PI / 180) *
                                         -sqrt(2 * pow(params_.end_pose.arm_extended_length / 2, 2)));
  l_hand_spline_.roll()->addPoint(time, 0);
  // angle has to be 45° due to arms being a Isosceles triangle with gamma = 90°
  l_hand_spline_.pitch()->addPoint(time, params_.dynup_back.arms_angle_back * M_PI / 180);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, sin(params_.dynup_back.arms_angle_back * M_PI / 180) *
                                         -sqrt(2 * pow(params_.end_pose.arm_extended_length / 2, 2)));
  r_hand_spline_.y()->addPoint(time, 0);
  r_hand_spline_.z()->addPoint(time, cos(params_.dynup_back.arms_angle_back * M_PI / 180) *
                                         -sqrt(2 * pow(params_.end_pose.arm_extended_length / 2, 2)));
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, params_.dynup_back.arms_angle_back * M_PI / 180);
  r_hand_spline_.yaw()->addPoint(time, 0);

  double angle_foot = M_PI * params_.dynup_back.foot_angle / 180;
  // trunk needs to be kept high enough to avoid collisions
  // since angle between torso and foot changes, we need to apply sin/cos to compute in relation to feet.
  // this is necessary since it will be the correct frame again after next torso rotation
  // shift torso by general x offset + extra parameter to allow positioning of CoM.
  r_foot_spline_.x()->addPoint(time, -params_.dynup_back.trunk_height_back);
  r_foot_spline_.y()->addPoint(time, -params_.end_pose.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -params_.dynup_back.com_shift_1);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, angle_foot);
  r_foot_spline_.yaw()->addPoint(time, 0);
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.end_pose.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);

  /*
   * Pose 3: Fully extend arms to the back, to push torso on the feet
   */
  time += params_.dynup_back.time_full_squat_hands;
  l_hand_spline_.x()->addPoint(time, -params_.end_pose.arm_extended_length);
  l_hand_spline_.y()->addPoint(time, 0);
  l_hand_spline_.z()->addPoint(time, 0);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, -params_.end_pose.arm_extended_length);
  r_hand_spline_.y()->addPoint(time, 0);
  r_hand_spline_.z()->addPoint(time, 0);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI);
  r_hand_spline_.yaw()->addPoint(time, 0);

  /*
   * Pose 4: Turn feet to correct end angle
   */
  time += params_.dynup_back.time_full_squat_legs;
  // angle foot now changed based on different parameter
  angle_foot = -M_PI * params_.dynup_back.trunk_overshoot_angle_back / 180;
  r_foot_spline_.x()->addPoint(time, -params_.dynup_back.com_shift_2);
  r_foot_spline_.y()->addPoint(time, -params_.end_pose.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -params_.dynup_back.trunk_height_back);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, angle_foot);
  r_foot_spline_.yaw()->addPoint(time, 0);
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.end_pose.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);

  /*
   * Pose 5: Wait in squat to let instabilities settle
   */
  time += params_.dynup_back.wait_in_squat_back;
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.end_pose.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);
  r_foot_spline_.x()->addPoint(time, -params_.dynup_back.com_shift_2);
  r_foot_spline_.y()->addPoint(time, -params_.end_pose.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -params_.dynup_back.trunk_height_back);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, M_PI * -params_.end_pose.trunk_pitch / 180);
  r_foot_spline_.yaw()->addPoint(time, 0);

  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, params_.end_pose.arm_side_offset_back);
  l_hand_spline_.z()->addPoint(time, -params_.end_pose.arm_extended_length);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI / 2);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -params_.end_pose.arm_side_offset_back);
  r_hand_spline_.z()->addPoint(time, -params_.end_pose.arm_extended_length);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI / 2);
  r_hand_spline_.yaw()->addPoint(time, 0);
  return time;
}

double DynupEngine::calcRiseSplines(double time) {
  // all positions relative to right foot
  // foot_trajectories_ are for left foot
  time += params_.rise.rise_time;
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.end_pose.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);

  r_foot_spline_.x()->addPoint(time, -params_.end_pose.trunk_x_final);
  r_foot_spline_.y()->addPoint(time, -params_.end_pose.foot_distance / 2.0);
  r_foot_spline_.z()->addPoint(time, -params_.end_pose.trunk_height);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, -params_.end_pose.trunk_pitch * M_PI / 180);
  r_foot_spline_.yaw()->addPoint(time, 0);

  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, 0);
  l_hand_spline_.z()->addPoint(time, params_.end_pose.hand_walkready_height);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, params_.end_pose.hand_walkready_pitch * M_PI / 180);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, 0);
  r_hand_spline_.z()->addPoint(time, params_.end_pose.hand_walkready_height);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, params_.end_pose.hand_walkready_pitch * M_PI / 180);
  r_hand_spline_.yaw()->addPoint(time, 0);

  return time;
}

double DynupEngine::calcDescendSplines(double time) {
  // all positions relative to right foot
  // foot_trajectories_ are for left foot
  time += params_.descend.descend_time;
  l_foot_spline_.x()->addPoint(time, 0);
  l_foot_spline_.y()->addPoint(time, params_.end_pose.foot_distance);
  l_foot_spline_.z()->addPoint(time, 0);
  l_foot_spline_.roll()->addPoint(time, 0);
  l_foot_spline_.pitch()->addPoint(time, 0);
  l_foot_spline_.yaw()->addPoint(time, 0);
  r_foot_spline_.x()->addPoint(time, -params_.end_pose.trunk_x_final);
  r_foot_spline_.y()->addPoint(time, -params_.end_pose.foot_distance / 2);
  r_foot_spline_.z()->addPoint(time, -params_.dynup_front.leg_min_length_front);
  r_foot_spline_.roll()->addPoint(time, 0);
  r_foot_spline_.pitch()->addPoint(time, M_PI * -params_.end_pose.trunk_pitch / 180);
  r_foot_spline_.yaw()->addPoint(time, 0);

  l_hand_spline_.x()->addPoint(time, 0);
  l_hand_spline_.y()->addPoint(time, params_.dynup_front.arm_side_offset_front);
  l_hand_spline_.z()->addPoint(time, -params_.end_pose.arm_extended_length);
  l_hand_spline_.roll()->addPoint(time, 0);
  l_hand_spline_.pitch()->addPoint(time, M_PI / 2);
  l_hand_spline_.yaw()->addPoint(time, 0);
  r_hand_spline_.x()->addPoint(time, 0);
  r_hand_spline_.y()->addPoint(time, -params_.dynup_front.arm_side_offset_front);
  r_hand_spline_.z()->addPoint(time, -params_.end_pose.arm_extended_length);
  r_hand_spline_.roll()->addPoint(time, 0);
  r_hand_spline_.pitch()->addPoint(time, M_PI / 2);
  r_hand_spline_.yaw()->addPoint(time, 0);

  return time;
}

void DynupEngine::setGoals(const DynupRequest &goals) {
  // we use hand splines from shoulder frame instead of base_link
  geometry_msgs::msg::Pose l_hand = goals.l_hand_pose;
  geometry_msgs::msg::Pose r_hand = goals.r_hand_pose;
  l_hand.position.y -= shoulder_offset_y_;
  l_hand.position.z -= arm_offset_z_;
  r_hand.position.y += shoulder_offset_y_;
  r_hand.position.z -= arm_offset_z_;
  // l_foot_spline_ is defined relative to r_foot_spline_, while all others are relative to base_link
  l_hand_spline_ = initializeSpline(l_hand, l_hand_spline_);
  r_hand_spline_ = initializeSpline(r_hand, r_hand_spline_);
  l_foot_spline_ = initializeSpline(goals.l_foot_pose, l_foot_spline_);
  r_foot_spline_ = initializeSpline(goals.r_foot_pose, r_foot_spline_);

  // get parameters from walking. If walking is not running, use default values
  // we re-request the values every time because they can be changed by dynamic reconfigure
  // and re-requesting them is fast enough
  std::vector<rclcpp::Parameter> walking_params;
  // Get params and wait for walking to be ready
  walking_params = walking_param_client_->get_parameters(
      {"engine.trunk_pitch", "engine.trunk_height", "engine.foot_distance", "engine.trunk_x_offset"},
      std::chrono::seconds(10));

  // when the walking was killed, service_is_ready is still true but the parameters come back empty
  if (walking_params.size() != 4) {
    RCLCPP_WARN(node_->get_logger(), "Walking is not running, using default parameters for walkready.");
  } else {
    // Placeholders for the parameters, we assure that we have all parameters before, so we can use -1 as a placeholder
    double foot_distance = -1, trunk_x_final = -1, trunk_pitch = -1, trunk_height = -1;
    for (auto &param : walking_params) {
      if (param.get_name() == "engine.trunk_pitch") {
        trunk_pitch = param.get_value<double>();
      } else if (param.get_name() == "engine.trunk_height") {
        trunk_height = param.get_value<double>();
      } else if (param.get_name() == "engine.foot_distance") {
        foot_distance = param.get_value<double>();
      } else if (param.get_name() == "engine.trunk_x_offset") {
        trunk_x_final = param.get_value<double>();
      }
    }

    // walking uses a different coordinate system for the trunk
    trunk_height = trunk_height * std::cos(trunk_pitch);
    trunk_x_final = trunk_x_final - std::sin(trunk_pitch) * trunk_height;
    params_.end_pose.trunk_pitch = trunk_pitch * 180 / M_PI;
    params_.end_pose.trunk_height = trunk_height;
    params_.end_pose.foot_distance = foot_distance;
    params_.end_pose.trunk_x_final = trunk_x_final;
  }

  if (goals.direction == "front") {
    // add front and rise splines together
    double time = calcFrontSplines();
    duration_ = calcRiseSplines(time);
    direction_ = 1;
  } else if (goals.direction == "back") {
    // add back and rise splines together
    double time = calcBackSplines();
    duration_ = calcRiseSplines(time);
    direction_ = 0;
  } else if (goals.direction == "front-only") {
    duration_ = calcFrontSplines();
    direction_ = 1;
  } else if (goals.direction == "back-only") {
    duration_ = calcBackSplines();
    direction_ = 0;
  } else if (goals.direction == "rise") {
    duration_ = calcRiseSplines(0);
    direction_ = 2;
  } else if (goals.direction == "descend") {
    duration_ = calcDescendSplines(0);
    direction_ = 3;
  } else if (goals.direction == "front_only") {
    duration_ = calcFrontSplines();
    direction_ = 4;
  } else if (goals.direction == "back_only") {
    duration_ = calcBackSplines();
    direction_ = 5;
  } else if (goals.direction == "walkready") {
    duration_ = calcRiseSplines(params_.dynup_front.time_walkready);
    direction_ = 6;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Provided direction not known");
  }
}

int DynupEngine::getPercentDone() const { return int(time_ / duration_ * 100); }

double DynupEngine::getDuration() const { return duration_; }

/*Calculates if we are at a point of the animation where stabilizing should be applied. */
bool DynupEngine::isStabilizingNeeded() {
  return ((direction_ == 1 && time_ >= params_.dynup_front.time_hands_side + params_.dynup_front.time_hands_rotate +
                                           params_.dynup_front.time_foot_close + params_.dynup_front.time_hands_front +
                                           params_.dynup_front.time_foot_ground_front +
                                           params_.dynup_front.time_torso_45 + params_.dynup_front.time_to_squat) ||
          (direction_ == 0 && time_ >= params_.dynup_back.time_legs_close + params_.dynup_back.time_foot_ground_back +
                                           params_.dynup_back.time_full_squat_hands +
                                           params_.dynup_back.time_full_squat_legs) ||
          (direction_ == 2) || (direction_ == 3));
}

bool DynupEngine::isHeadZero() {
  // set heads zero in the middle of rise phase
  return ((direction_ == 1 && time_ >= params_.dynup_front.time_hands_side + params_.dynup_front.time_hands_rotate +
                                           params_.dynup_front.time_foot_close + params_.dynup_front.time_hands_front +
                                           params_.dynup_front.time_foot_ground_front +
                                           params_.dynup_front.time_torso_45 + params_.dynup_front.time_to_squat +
                                           params_.dynup_front.wait_in_squat_front + 0.5 * params_.rise.rise_time) ||
          (direction_ == 0 && time_ >= params_.dynup_back.time_legs_close + params_.dynup_back.time_foot_ground_back +
                                           params_.dynup_back.time_full_squat_hands +
                                           params_.dynup_back.time_full_squat_legs +
                                           params_.dynup_back.wait_in_squat_back + 0.5 * params_.rise.rise_time) ||
          (direction_ == 2) || (direction_ == 3));
}

bitbots_splines::PoseSpline DynupEngine::getRFootSplines() const { return r_foot_spline_; }

bitbots_splines::PoseSpline DynupEngine::getLFootSplines() const { return l_foot_spline_; }

bitbots_splines::PoseSpline DynupEngine::getRHandSplines() const { return r_hand_spline_; }

bitbots_splines::PoseSpline DynupEngine::getLHandSplines() const { return l_hand_spline_; }

void DynupEngine::setParams(bitbots_dynup::Params::Engine params) {
  params_ = params;
  arm_offset_y_ = shoulder_offset_y_;
  offset_left_ = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), {0, arm_offset_y_, arm_offset_z_});
  offset_right_ = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), {0, -arm_offset_y_, arm_offset_z_});
}

int DynupEngine::getDirection() { return direction_; }

}  // namespace bitbots_dynup
