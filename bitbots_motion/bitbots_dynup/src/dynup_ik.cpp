#include <bitbots_dynup/dynup_ik.hpp>
namespace bitbots_dynup {

DynupIK::DynupIK(rclcpp::Node::SharedPtr node) : node_(node) {}

void DynupIK::init(moveit::core::RobotModelPtr kinematic_model) {
  current_joint_states_ = std::make_shared<sensor_msgs::msg::JointState>();
  /* Extract joint groups from kinematics model */
  l_leg_joints_group_ = kinematic_model->getJointModelGroup("LeftLeg");
  r_leg_joints_group_ = kinematic_model->getJointModelGroup("RightLeg");
  l_arm_joints_group_ = kinematic_model->getJointModelGroup("LeftArm");
  r_arm_joints_group_ = kinematic_model->getJointModelGroup("RightArm");
  all_joints_group_ = kinematic_model->getJointModelGroup("All");

  /* Reset kinematic goal to default */
  goal_state_.reset(new moveit::core::RobotState(kinematic_model));
  goal_state_->setToDefaultValues();
}

void DynupIK::reset() {
  // we have to set some good initial position in the goal state, since we are using a gradient
  // based method. Otherwise, the first step will be not correct
  std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
  std::vector<double> pos_vec = {0.7, 1.0, -0.4, -0.7, -1.0, 0.4};
  for (size_t i = 0; i < names_vec.size(); i++) {
    // besides its name, this method only changes a single joint position...
    goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
  }
}

void DynupIK::setDirection(std::string direction) { direction_ = direction; }

bitbots_splines::JointGoals DynupIK::calculate(const DynupResponse &ik_goals) { // What is DynupResponse? liegt in dynup_utils.h und ist ein struct
  /* ik options is basically the command which we send to bio_ik and which describes what we want to do */
  auto ik_options = kinematics::KinematicsQueryOptions();
  ik_options.return_approximate_solution = true;

  geometry_msgs::msg::Pose right_foot_goal_msg, left_foot_goal_msg, right_hand_goal_msg, left_hand_goal_msg;

  tf2::toMsg(ik_goals.r_foot_goal_pose, right_foot_goal_msg);
  tf2::toMsg(ik_goals.l_foot_goal_pose, left_foot_goal_msg);
  tf2::toMsg(ik_goals.r_hand_goal_pose, right_hand_goal_msg);
  tf2::toMsg(ik_goals.l_hand_goal_pose, left_hand_goal_msg);

  bool success;
  goal_state_->updateLinkTransforms();

  success = goal_state_->setFromIK(l_leg_joints_group_, left_foot_goal_msg, 0.005,
                                   moveit::core::GroupStateValidityCallbackFn(), ik_options);

  goal_state_->updateLinkTransforms();

  success &= goal_state_->setFromIK(r_leg_joints_group_, right_foot_goal_msg, 0.005,
                                    moveit::core::GroupStateValidityCallbackFn(), ik_options);

  goal_state_->updateLinkTransforms();

  success &= goal_state_->setFromIK(l_arm_joints_group_, left_hand_goal_msg, 0.005,
                                    moveit::core::GroupStateValidityCallbackFn(), ik_options);

  goal_state_->updateLinkTransforms();

  success &= goal_state_->setFromIK(r_arm_joints_group_, right_hand_goal_msg, 0.005,
                                    moveit::core::GroupStateValidityCallbackFn(), ik_options);
  if (success) {
    /* retrieve joint names and associated positions from  */
    std::vector<std::string> joint_names = all_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(all_joints_group_, joint_goals);

    /* construct result object */
    bitbots_splines::JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;
    /* sets head motors to correct positions, as the IK will return random values for those unconstrained motors. */
    for (size_t i = result.first.size(); i-- > 0;) {
      if (result.first[i] == "HeadPan") {
        if (direction_ == "walkready") {
          // remove head from the goals so that we can move it freely
          result.first.erase(result.first.begin() + i);
          result.second.erase(result.second.begin() + i);
        } else {
          result.second[i] = 0;
        }
      } else if (result.first[i] == "HeadTilt") {
        if (ik_goals.is_head_zero) {
          result.second[i] = 0;
        } else {
          if (direction_ == "front") {
            result.second[i] = 1.0;
          } else if (direction_ == "back") {
            result.second[i] = -1.5;
          } else if (direction_ == "walkready") {
            // remove head from the goals so that we can move it freely
            result.first.erase(result.first.begin() + i);
            result.second.erase(result.second.begin() + i);
          } else {
            result.second[i] = 0;
          }
        }
      }
    }
    return result;
  } else {
    // node will count this as a missing tick and provide warning
    return {};
  }
}
void DynupIK::useStabilizing(bool use) { use_stabilizing_ = use; }

void DynupIK::setCurrentJointStates(sensor_msgs::msg::JointState::SharedPtr jointStates) {
  current_joint_states_ = jointStates;
}

moveit::core::RobotStatePtr DynupIK::get_goal_state() { return goal_state_; }

}  // namespace bitbots_dynup
