#include <bitbots_dynup/dynup_ik.hpp>

namespace bitbots_dynup {

DynupIK::DynupIK(rclcpp::Node::SharedPtr node) : node_(node) {}

void DynupIK::init(moveit::core::RobotModelPtr kinematic_model) {
  /* Extract joint groups from kinematics model */
  l_leg_joints_group_ = kinematic_model->getJointModelGroup("LeftLeg");
  r_leg_joints_group_ = kinematic_model->getJointModelGroup("RightLeg");
  l_arm_joints_group_ = kinematic_model->getJointModelGroup("LeftArm");
  r_arm_joints_group_ = kinematic_model->getJointModelGroup("RightArm");
  all_joints_group_ = kinematic_model->getJointModelGroup("All");

  /* Reset kinematic goal to default */
  goal_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model);
  goal_state_->setToDefaultValues();
}

void DynupIK::reset() {
  // we have to set some good initial position in the goal state, since we are using a gradient
  // based method. Otherwise, the first step will be not correct

  // Use the current joint state as a starting point for the IK
  if (joint_state_) {
    goal_state_->setVariablePositions(joint_state_->name, joint_state_->position);
  } else {
    // if we don't have a joint state, we set some default values that are a reasonable starting point
    RCLCPP_WARN(node_->get_logger(),
                "No joint state received, using hardcoded initial positions for IK "
                "initialization");
    std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
    std::vector<double> pos_vec = {0.7, 1.0, -0.4, -0.7, -1.0, 0.4};
    for (size_t i = 0; i < names_vec.size(); i++) {
      // besides its name, this method only changes a single joint position...
      goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
    }
  }
}

void DynupIK::setDirection(DynupDirection direction) { direction_ = direction; }

bitbots_splines::JointGoals DynupIK::calculate(const DynupResponse& ik_goals) {
  /* ik options is basically the command which we send to bio_ik and which describes what we want to do */
  kinematics::KinematicsQueryOptions ik_options;
  ik_options.return_approximate_solution = true;

  geometry_msgs::msg::Pose right_foot_goal_msg, left_foot_goal_msg, right_hand_goal_msg, left_hand_goal_msg;

  tf2::toMsg(ik_goals.r_foot_goal_pose, right_foot_goal_msg);
  tf2::toMsg(ik_goals.l_foot_goal_pose, left_foot_goal_msg);
  tf2::toMsg(ik_goals.r_hand_goal_pose, right_hand_goal_msg);
  tf2::toMsg(ik_goals.l_hand_goal_pose, left_hand_goal_msg);

  bool success;
  goal_state_->updateLinkTransforms();

  bio_ik::BioIKKinematicsQueryOptions leg_ik_options;
  leg_ik_options.return_approximate_solution = true;

  // Add auxiliary goal to prevent bending the knees in the wrong direction when we go from init to walkready
  leg_ik_options.goals.push_back(std::make_unique<bio_ik::AvoidJointLimitsGoal>());

  success = goal_state_->setFromIK(l_leg_joints_group_, left_foot_goal_msg, 0.005,
                                   moveit::core::GroupStateValidityCallbackFn(), leg_ik_options);

  goal_state_->updateLinkTransforms();

  success &= goal_state_->setFromIK(r_leg_joints_group_, right_foot_goal_msg, 0.005,
                                    moveit::core::GroupStateValidityCallbackFn(), leg_ik_options);

  if (direction_ != DynupDirection::RISE_NO_ARMS and direction_ != DynupDirection::DESCEND_NO_ARMS) {
    goal_state_->updateLinkTransforms();
    success &= goal_state_->setFromIK(l_arm_joints_group_, left_hand_goal_msg, 0.005,
                                      moveit::core::GroupStateValidityCallbackFn(), ik_options);

    goal_state_->updateLinkTransforms();
    success &= goal_state_->setFromIK(r_arm_joints_group_, right_hand_goal_msg, 0.005,
                                      moveit::core::GroupStateValidityCallbackFn(), ik_options);
  }
  if (success) {
    /* retrieve joint names and associated positions from  */
    auto joint_names = all_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(all_joints_group_, joint_goals);

    /* construct result object */
    bitbots_splines::JointGoals result = {joint_names, joint_goals};

    // Store the name of the arm joins so we can remove them if they are not needed
    const auto r_arm_motors = r_arm_joints_group_->getActiveJointModelNames();
    const auto l_arm_motors = l_arm_joints_group_->getActiveJointModelNames();

    /* sets head motors to correct positions, as the IK will return random values for those unconstrained motors. */
    for (size_t i = result.first.size(); i-- > 0;) {
      if (result.first[i] == "HeadPan") {
        if (direction_ == DynupDirection::WALKREADY) {
          // remove head from the goals so that we can move it freely
          result.first.erase(result.first.begin() + i);
          result.second.erase(result.second.begin() + i);
        } else {
          result.second[i] = 0;
        }
      } else if (result.first[i] == "HeadTilt") {
        if (ik_goals.is_head_zero) {
          result.second[i] = 0;
        } else if (direction_ == DynupDirection::FRONT or direction_ == DynupDirection::FRONT_ONLY) {
          result.second[i] = 1.0;
        } else if (direction_ == DynupDirection::BACK or direction_ == DynupDirection::BACK_ONLY) {
          result.second[i] = -1.5;
        } else if (direction_ == DynupDirection::WALKREADY) {
          // remove head from the goals so that we can move it freely
          result.first.erase(result.first.begin() + i);
          result.second.erase(result.second.begin() + i);
        } else {
          result.second[i] = 0;
        }
      }
      // Remove the arm motors from the goals if we have a goal without arms
      else if ((std::find(r_arm_motors.begin(), r_arm_motors.end(), result.first[i]) != r_arm_motors.end() or
                std::find(l_arm_motors.begin(), l_arm_motors.end(), result.first[i]) != l_arm_motors.end()) and
               (direction_ == DynupDirection::RISE_NO_ARMS or direction_ == DynupDirection::DESCEND_NO_ARMS)) {
        result.first.erase(result.first.begin() + i);
        result.second.erase(result.second.begin() + i);
      }
    }
    return result;
  } else {
    // node will count this as a missing tick and provide warning
    return {};
  }
}

moveit::core::RobotStatePtr DynupIK::get_goal_state() { return goal_state_; }

void DynupIK::set_joint_positions(sensor_msgs::msg::JointState::ConstSharedPtr joint_state) {
  joint_state_ = joint_state;
}
}  // namespace bitbots_dynup
