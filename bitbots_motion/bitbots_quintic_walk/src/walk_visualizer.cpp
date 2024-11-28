#include "bitbots_quintic_walk/walk_visualizer.hpp"

namespace bitbots_quintic_walk {
WalkVisualizer::WalkVisualizer(rclcpp::Node::SharedPtr node, walking::Params::Node::Tf tf_config)
    : node_(node),
      tf_config_(tf_config),
      pub_debug_(node_->create_publisher<bitbots_quintic_walk::msg::WalkDebug>("walk_debug", 1)),
      pub_engine_debug_(node_->create_publisher<bitbots_quintic_walk::msg::WalkEngineDebug>("walk_engine_debug", 1)),
      pub_debug_marker_(node_->create_publisher<visualization_msgs::msg::MarkerArray>("walk_debug_marker", 1)) {}

void WalkVisualizer::init(moveit::core::RobotModelPtr kinematic_model) { kinematic_model_ = kinematic_model; }

void WalkVisualizer::publishDebug(const WalkResponse &current_response,
                                  const moveit::core::RobotStatePtr &current_state,
                                  const bitbots_splines::JointGoals &motor_goals) {
  visualization_msgs::msg::MarkerArray marker_array;

  auto [engine_debug, engine_markers] = publishEngineDebug(current_response);
  marker_array.markers.insert(marker_array.markers.end(), engine_markers.markers.begin(), engine_markers.markers.end());
  pub_engine_debug_->publish(engine_debug);

  auto [ik_debug, ik_markers] = publishIKDebug(current_response, current_state, motor_goals);
  marker_array.markers.insert(marker_array.markers.end(), ik_markers.markers.begin(), ik_markers.markers.end());
  pub_debug_->publish(ik_debug);

  auto walk_markers = publishWalkMarkers(current_response);
  marker_array.markers.insert(marker_array.markers.end(), walk_markers.markers.begin(), walk_markers.markers.end());

  pub_debug_marker_->publish(marker_array);
}

std::pair<bitbots_quintic_walk::msg::WalkEngineDebug, visualization_msgs::msg::MarkerArray>
WalkVisualizer::publishEngineDebug(WalkResponse response) {
  // Here we will convert the walk engine response to a various debug messages and RViz markers
  // Initialize output containers
  bitbots_quintic_walk::msg::WalkEngineDebug msg;
  visualization_msgs::msg::MarkerArray marker_array;

  // Copy some data into the debug message
  msg.is_left_support = response.is_left_support_foot;
  msg.is_double_support = response.is_double_support;
  msg.header.stamp = node_->now();
  msg.phase_time = response.phase;
  msg.traj_time = response.traj_time;
  // Copy walk engine state
  static const std::unordered_map<WalkState, std::string> state_string_mapping = {
      {WalkState::IDLE, "idle"},
      {WalkState::START_MOVEMENT, "start_movement"},
      {WalkState::START_STEP, "start_step"},
      {WalkState::WALKING, "walking"},
      {WalkState::PAUSED, "paused"},
      {WalkState::KICK, "kick"},
      {WalkState::STOP_STEP, "stop_step"},
      {WalkState::STOP_MOVEMENT, "stop_movement"}};
  msg.state.data = state_string_mapping.at(response.state);
  msg.state_number = static_cast<int>(response.state);

  // Define current support foot frame
  std::string current_support_frame;
  if (response.is_left_support_foot) {
    current_support_frame = tf_config_.l_sole_frame;
  } else {
    current_support_frame = tf_config_.r_sole_frame;
  }

  // Create placeholder floats
  double _1, _2;
  // Copy transform of the last footstep position (and orientation) to the debug message
  msg.footstep_last.x = response.support_to_last.getOrigin()[0];
  msg.footstep_last.y = response.support_to_last.getOrigin()[1];
  tf2::Matrix3x3(response.support_to_last.getRotation()).getRPY(_1, _2, msg.footstep_last.z);

  // Copy transform of the next footstep position (and orientation) to the debug message
  msg.footstep_next.x = response.support_to_next.getOrigin()[0];
  msg.footstep_next.y = response.support_to_next.getOrigin()[1];
  tf2::Matrix3x3(response.support_to_next.getRotation()).getRPY(_1, _2, msg.footstep_next.z);

  // Copy cartesian coordinates of the currently flying foot relative to the support foot to the debug message
  tf2::toMsg(response.support_foot_to_flying_foot, msg.fly_goal);
  // Create an additional marker for the flying foot goal
  marker_array.markers.push_back(createArrowMarker("engine_fly_goal", current_support_frame, msg.fly_goal, BLUE));
  RCLCPP_INFO_ONCE(node_->get_logger(),
                   "Color for the Engine Debug marker, showing where the flying foot and trunk should be, is blue!");

  // Copy the rotation of the flying foot relative to the support foot to the debug message
  tf2::Matrix3x3(response.support_foot_to_flying_foot.getRotation())
      .getRPY(msg.fly_euler.x, msg.fly_euler.y, msg.fly_euler.z);

  // Copy cartesian coordinates of the trunk goal relative to the support foot to the debug message
  tf2::toMsg(response.support_foot_to_trunk, msg.trunk_goal);
  // Create an additional marker for the trunk goal
  marker_array.markers.push_back(createArrowMarker("engine_trunk_goal", current_support_frame, msg.trunk_goal, BLUE));

  // TODO check this!!!
  msg.trunk_goal_abs = msg.trunk_goal;
  if (msg.trunk_goal_abs.position.y > 0) {
    msg.trunk_goal_abs.position.y -= response.foot_distance / 2;
  } else {
    msg.trunk_goal_abs.position.y += response.foot_distance / 2;
  }

  tf2::Matrix3x3(response.support_foot_to_flying_foot.getRotation())
      .getRPY(msg.trunk_euler.x, msg.trunk_euler.y, msg.trunk_euler.z);

  // resulting trunk pose
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1;
  marker_array.markers.push_back(createArrowMarker("trunk_result", tf_config_.base_link_frame, pose, GREEN));
  RCLCPP_INFO_ONCE(node_->get_logger(), "Color for the Engine Debug marker, showing where the trunk is, is green!");

  return {msg, marker_array};
}

std::pair<bitbots_quintic_walk::msg::WalkDebug, visualization_msgs::msg::MarkerArray> WalkVisualizer::publishIKDebug(
    WalkResponse response, moveit::core::RobotStatePtr current_state, bitbots_splines::JointGoals joint_goals) {
  bitbots_quintic_walk::msg::WalkDebug msg;
  visualization_msgs::msg::MarkerArray marker_array;

  tf2::Transform trunk_to_support_foot = response.support_foot_to_trunk.inverse();
  tf2::Transform trunk_to_flying_foot = trunk_to_support_foot * response.support_foot_to_flying_foot;

  // Copy goals into the message
  tf2::toMsg(trunk_to_support_foot, msg.support_foot_goal);
  tf2::toMsg(trunk_to_flying_foot, msg.fly_foot_goal);
  // Set left and right foot goals based on the support foot
  if (response.is_left_support_foot) {
    msg.left_foot_goal = msg.support_foot_goal;
    msg.right_foot_goal = msg.fly_foot_goal;
  } else {
    msg.left_foot_goal = msg.fly_foot_goal;
    msg.right_foot_goal = msg.support_foot_goal;
  }
  // Create additional markers for the foot goals
  marker_array.markers.push_back(
      createArrowMarker("engine_left_goal", tf_config_.base_link_frame, msg.left_foot_goal, ORANGE));
  marker_array.markers.push_back(
      createArrowMarker("engine_right_goal", tf_config_.base_link_frame, msg.right_foot_goal, ORANGE));
  RCLCPP_INFO_ONCE(node_->get_logger(), "Color for the IK marker, showing where the feet should be, is orange!");

  // IK results
  moveit::core::RobotStatePtr goal_state;
  goal_state.reset(new moveit::core::RobotState(kinematic_model_));
  auto &[names, goals] = joint_goals;
  for (size_t i = 0; i < names.size(); i++) {
    // besides its name, this method only changes a single joint position...
    goal_state->setJointPositions(names[i], &goals[i]);
  }
  goal_state->updateLinkTransforms();
  tf2::convert(goal_state->getFrameTransform("l_sole"), msg.left_foot_ik_result);
  tf2::convert(goal_state->getFrameTransform("r_sole"), msg.right_foot_ik_result);
  if (response.is_left_support_foot) {
    msg.support_foot_ik_result = msg.left_foot_ik_result;
    msg.fly_foot_ik_result = msg.right_foot_ik_result;
  } else {
    msg.support_foot_ik_result = msg.right_foot_ik_result;
    msg.fly_foot_ik_result = msg.left_foot_ik_result;
  }
  marker_array.markers.push_back(
      createArrowMarker("ik_left", tf_config_.base_link_frame, msg.left_foot_ik_result, GREEN));
  marker_array.markers.push_back(
      createArrowMarker("ik_right", tf_config_.base_link_frame, msg.right_foot_ik_result, GREEN));
  RCLCPP_INFO_ONCE(node_->get_logger(), "Color for the IK marker, showing the ik result, is green!");

  // IK offsets
  tf2::Vector3 support_off;
  tf2::Vector3 fly_off;
  tf2::Vector3 tf_vec_left;
  tf2::Vector3 tf_vec_right;
  Eigen::Vector3d l_transform = goal_state->getGlobalLinkTransform("l_sole").translation();
  Eigen::Vector3d r_transform = goal_state->getGlobalLinkTransform("r_sole").translation();
  tf2::convert(l_transform, tf_vec_left);
  tf2::convert(r_transform, tf_vec_right);
  geometry_msgs::msg::Vector3 vect_msg;
  if (response.is_left_support_foot) {
    support_off = trunk_to_support_foot.getOrigin() - tf_vec_left;
    fly_off = trunk_to_flying_foot.getOrigin() - tf_vec_right;
    vect_msg.x = support_off.x();
    vect_msg.y = support_off.y();
    vect_msg.z = support_off.z();
    msg.left_foot_ik_offset = vect_msg;
    vect_msg.x = fly_off.x();
    vect_msg.y = fly_off.y();
    vect_msg.z = fly_off.z();
    msg.right_foot_ik_offset = vect_msg;
  } else {
    support_off = trunk_to_support_foot.getOrigin() - tf_vec_right;
    fly_off = trunk_to_flying_foot.getOrigin() - tf_vec_left;
    vect_msg.x = fly_off.x();
    vect_msg.y = fly_off.y();
    vect_msg.z = fly_off.z();
    msg.left_foot_ik_offset = vect_msg;
    vect_msg.x = support_off.x();
    vect_msg.y = support_off.y();
    vect_msg.z = support_off.z();
    msg.right_foot_ik_offset = vect_msg;
  }
  vect_msg.x = support_off.x();
  vect_msg.y = support_off.y();
  vect_msg.z = support_off.z();
  msg.support_foot_ik_offset = vect_msg;
  vect_msg.x = fly_off.x();
  vect_msg.y = fly_off.y();
  vect_msg.z = fly_off.z();
  msg.fly_foot_ik_offset = vect_msg;

  // Actual foot positions determined by the IK solver (not strictly equal to the goals)
  tf2::convert(current_state->getGlobalLinkTransform("l_sole"), msg.left_foot_position);
  tf2::convert(current_state->getGlobalLinkTransform("r_sole"), msg.right_foot_position);
  if (response.is_left_support_foot) {
    msg.support_foot_position = msg.left_foot_position;
    msg.fly_foot_position = msg.right_foot_position;
  } else {
    msg.support_foot_position = msg.right_foot_position;
    msg.fly_foot_position = msg.left_foot_position;
  }

  // Calculate offsets between the actual foot positions and the goals (meaning the IK solver error)
  l_transform = current_state->getGlobalLinkTransform("l_sole").translation();
  r_transform = current_state->getGlobalLinkTransform("r_sole").translation();
  tf2::convert(l_transform, tf_vec_left);
  tf2::convert(r_transform, tf_vec_right);
  if (response.is_left_support_foot) {
    support_off = trunk_to_support_foot.getOrigin() - tf_vec_left;
    fly_off = trunk_to_flying_foot.getOrigin() - tf_vec_right;
    vect_msg.x = support_off.x();
    vect_msg.y = support_off.y();
    vect_msg.z = support_off.z();
    msg.left_foot_actual_offset = vect_msg;
    vect_msg.x = fly_off.x();
    vect_msg.y = fly_off.y();
    vect_msg.z = fly_off.z();
    msg.right_foot_actual_offset = vect_msg;
  } else {
    support_off = trunk_to_support_foot.getOrigin() - tf_vec_right;
    fly_off = trunk_to_flying_foot.getOrigin() - tf_vec_left;
    vect_msg.x = fly_off.x();
    vect_msg.y = fly_off.y();
    vect_msg.z = fly_off.z();
    msg.left_foot_actual_offset = vect_msg;
    vect_msg.x = support_off.x();
    vect_msg.y = support_off.y();
    vect_msg.z = support_off.z();
    msg.right_foot_actual_offset = vect_msg;
  }
  vect_msg.x = support_off.x();
  vect_msg.y = support_off.y();
  vect_msg.z = support_off.z();
  msg.support_foot_actual_offset = vect_msg;
  vect_msg.x = fly_off.x();
  vect_msg.y = fly_off.y();
  vect_msg.z = fly_off.z();
  msg.fly_foot_actual_offset = vect_msg;

  return {msg, marker_array};
}

visualization_msgs::msg::Marker WalkVisualizer::createArrowMarker(const std::string &name_space,
                                                                  const std::string &frame,
                                                                  const geometry_msgs::msg::Pose &pose,
                                                                  const std_msgs::msg::ColorRGBA &color) {
  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header.frame_id = frame;

  marker_msg.type = marker_msg.ARROW;
  marker_msg.ns = name_space;
  marker_msg.action = marker_msg.ADD;
  marker_msg.pose = pose;
  marker_msg.color = color;

  geometry_msgs::msg::Vector3 scale;
  scale.x = 0.01;
  scale.y = 0.003;
  scale.z = 0.003;
  marker_msg.scale = scale;
  marker_msg.id = 0;

  return marker_msg;
}

visualization_msgs::msg::MarkerArray WalkVisualizer::publishWalkMarkers(WalkResponse response) {
  visualization_msgs::msg::MarkerArray marker_array;

  // Create a marker for the last step
  visualization_msgs::msg::Marker support_foot_marker_msg;
  if (response.is_left_support_foot) {
    support_foot_marker_msg.header.frame_id = tf_config_.l_sole_frame;
  } else {
    support_foot_marker_msg.header.frame_id = tf_config_.r_sole_frame;
  }
  support_foot_marker_msg.type = support_foot_marker_msg.CUBE;
  support_foot_marker_msg.action = 0;
  support_foot_marker_msg.lifetime = rclcpp::Duration::from_nanoseconds(0.0);
  support_foot_marker_msg.scale.x = 0.2;
  support_foot_marker_msg.scale.y = 0.1;
  support_foot_marker_msg.scale.z = 0.01;
  support_foot_marker_msg.ns = "last_step";
  support_foot_marker_msg.id = 1;
  support_foot_marker_msg.color = BLACK;
  support_foot_marker_msg.color.a = 0.5;
  tf2::toMsg(response.support_to_last, support_foot_marker_msg.pose);
  marker_array.markers.push_back(support_foot_marker_msg);

  // This step center
  auto step_center_marker_msg(support_foot_marker_msg);
  step_center_marker_msg.ns = "step_center";
  step_center_marker_msg.id = 2;
  step_center_marker_msg.scale.x = 0.01;
  step_center_marker_msg.scale.y = 0.01;
  step_center_marker_msg.scale.z = 0.01;
  marker_array.markers.push_back(step_center_marker_msg);

  // Next step
  auto next_step_marker_msg(support_foot_marker_msg);
  next_step_marker_msg.id = 3;
  next_step_marker_msg.ns = "next_step";
  next_step_marker_msg.scale.x = 0.20;
  next_step_marker_msg.scale.y = 0.10;
  next_step_marker_msg.scale.z = 0.01;
  next_step_marker_msg.color = WHITE;
  next_step_marker_msg.color.a = 0.5;
  tf2::toMsg(response.support_to_next, next_step_marker_msg.pose);
  marker_array.markers.push_back(next_step_marker_msg);

  return marker_array;
}

}  // namespace bitbots_quintic_walk
