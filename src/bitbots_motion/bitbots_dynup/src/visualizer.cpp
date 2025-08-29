#include <bitbots_dynup/visualizer.hpp>
#include <bitbots_splines/pose_spline.hpp>

namespace bitbots_dynup {
Visualizer::Visualizer(rclcpp::Node::SharedPtr node, bitbots_dynup::Params::Visualizer params,
                       const std::string &base_topic)
    : node_(node), base_topic_(base_topic), params_(params) {
  /* make sure base_topic_ has consistent scheme */
  if (base_topic.compare(base_topic.size() - 1, 1, "/") != 0) {
    base_topic_ += "/";
  }

  /* create necessary publishers */
  spline_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(base_topic_ + "received_goal",
                                                                                    /* queue_size */ 5);
  ik_debug_publisher_ = node_->create_publisher<bitbots_dynup::msg::DynupIkOffset>(base_topic_ + "ik_debug", 5);
}

void Visualizer::setParams(bitbots_dynup::Params::Visualizer params) { params_ = params; }

void Visualizer::displaySplines(bitbots_splines::PoseSpline splines, const std::string &frame) {
  // if (spline_publisher_->get_subscription_count() == 0)
  //     return;
  visualization_msgs::msg::MarkerArray path = getPath(splines, frame, params_.spline_smoothness, node_);

  spline_publisher_->publish(path);
}

void Visualizer::publishIKOffsets(const moveit::core::RobotModelPtr &model, const DynupResponse &response,
                                  const bitbots_splines::JointGoals &ik_joint_goals) {
  bitbots_dynup::msg::DynupIkOffset msg;
  // those are the motor goals for the robot computed by the IK
  moveit::core::RobotStatePtr ik_state = std::make_shared<moveit::core::RobotState>(model);
  std::vector<std::string> names = ik_joint_goals.first;
  std::vector<double> goals = ik_joint_goals.second;
  for (size_t i = 0; i < names.size(); i++) {
    // besides its name, this method only changes a single joint position...
    ik_state->setJointPositions(names[i], &goals[i]);
  }

  ik_state->updateLinkTransforms();

  // set it for r_wrist, l_wirst, l_sole, r_sole
  // that is how you get an const Eigen::Affine3d of the pose of the link
  // after we did this, we obtain a position and orientation of the link in the world frame?
  Eigen::Isometry3d pose_right_result = ik_state->getFrameTransform("r_sole");
  Eigen::Isometry3d pose_left_result = ik_state->getFrameTransform("l_sole");
  Eigen::Isometry3d pose_right_hand = ik_state->getFrameTransform("r_wrist");
  Eigen::Isometry3d pose_left_hand = ik_state->getFrameTransform("l_wrist");

  Eigen::Isometry3d pose_right_foot_goal = tf2::transformToEigen(tf2::toMsg(response.r_foot_goal_pose));
  Eigen::Isometry3d pose_left_foot_goal = tf2::transformToEigen(tf2::toMsg(response.l_foot_goal_pose));
  Eigen::Isometry3d pose_right_hand_goal = tf2::transformToEigen(tf2::toMsg(response.r_hand_goal_pose));
  Eigen::Isometry3d pose_left_hand_goal = tf2::transformToEigen(tf2::toMsg(response.l_hand_goal_pose));

  // caluclate differences
  Eigen::Vector3d diff_right_foot = pose_right_result.translation() - pose_right_foot_goal.translation();
  Eigen::Vector3d diff_left_foot = pose_left_result.translation() - pose_left_foot_goal.translation();
  Eigen::Vector3d diff_right_hand = pose_right_hand.translation() - pose_right_hand_goal.translation();
  Eigen::Vector3d diff_left_hand = pose_left_hand.translation() - pose_left_hand_goal.translation();

  // fill msg
  msg.right_foot_ik_offset.position = tf2::toMsg(diff_right_foot);
  msg.left_foot_ik_offset.position = tf2::toMsg(diff_left_foot);
  msg.right_hand_ik_offset.position = tf2::toMsg(diff_right_hand);
  msg.left_hand_ik_offset.position = tf2::toMsg(diff_left_hand);

  // publish message
  ik_debug_publisher_->publish(msg);
}
}  // namespace bitbots_dynup
