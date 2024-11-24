#define M_TAU M_PI * 2

#include <bitbots_quintic_walk/walk_node.hpp>
#include <iostream>
#include <memory>
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace bitbots_quintic_walk {

WalkNode::WalkNode(rclcpp::Node::SharedPtr node, const std::string& ns,
                   const std::vector<rclcpp::Parameter>& moveit_parameters)
    : node_(node),
      param_listener_(node_),
      config_(param_listener_.get_params()),
      walk_engine_(node_, config_.engine),
      stabilizer_(node_),
      ik_(node_, config_.node.ik),
      visualizer_(node_, config_.node.tf) {
  // Create dummy node for moveit. This is necessary for dynamic reconfigure to work (moveit does some bullshit with
  // parameter declarations, so we need to isolate the walking parameters from the moveit parameters).
  // If the walking is instantiated using the python wrapper, moveit parameters are passed because no moveit config
  // is loaded in the conventional way. Normally the moveit config is loaded via launch file and the passed vector is
  // empty.
  auto moveit_node = std::make_shared<rclcpp::Node>(
      "walking_moveit_node", ns,
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).parameter_overrides(
          moveit_parameters));
  // get all kinematics parameters from the move_group node if they are not set manually via constructor
  std::string check_kinematic_parameters;
  if (!moveit_node->get_parameter("robot_description_kinematics.LeftLeg.kinematics_solver",
                                  check_kinematic_parameters)) {
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_, "/move_group");
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        break;
      }
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10 * 1e9,
                           "Can't copy parameters from move_group node. Service not available, waiting again...");
    }
    rcl_interfaces::msg::ListParametersResult parameter_list =
        parameters_client->list_parameters({"robot_description_kinematics"}, 10);
    auto copied_parameters = parameters_client->get_parameters(parameter_list.names);
    for (auto& parameter : copied_parameters) {
      moveit_node->declare_parameter(parameter.get_name(), parameter.get_type());
      moveit_node->set_parameter(parameter);
    }
  }

  // Load MoveIt! model
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(moveit_node, "robot_description");

  kinematic_model_ = robot_model_loader_->getModel();

  if (!kinematic_model_) {
    RCLCPP_FATAL(node_->get_logger(), "No robot model loaded, killing quintic walk.");
    exit(1);
  }

  // Process the parameters
  updateParams();

  /* init publisher and subscriber */
  pub_controller_command_ = node_->create_publisher<bitbots_msgs::msg::JointCommand>("walking_motor_goals", 1);
  pub_odometry_ = node_->create_publisher<nav_msgs::msg::Odometry>("walk_engine_odometry", 1);
  pub_support_ = node_->create_publisher<biped_interfaces::msg::Phase>("walk_support_state", 1);
  step_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("step", 1, std::bind(&WalkNode::stepCb, this, _1));
  cmd_vel_sub_ =
      node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&WalkNode::cmdVelCb, this, _1));
  robot_state_sub_ = node_->create_subscription<bitbots_msgs::msg::RobotControlState>(
      "robot_state", 1, std::bind(&WalkNode::robotStateCb, this, _1));
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 1, std::bind(&WalkNode::jointStateCb, this, _1));
  kick_sub_ = node_->create_subscription<std_msgs::msg::Bool>("kick", 1, std::bind(&WalkNode::kickCb, this, _1));
  imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, std::bind(&WalkNode::imuCb, this, _1));
  pressure_sub_left_ = node_->create_subscription<bitbots_msgs::msg::FootPressure>(
      "foot_pressure_left/filtered", 1, std::bind(&WalkNode::pressureLeftCb, this, _1));
  pressure_sub_right_ = node_->create_subscription<bitbots_msgs::msg::FootPressure>(
      "foot_pressure_right/filtered", 1, std::bind(&WalkNode::pressureRightCb, this, _1));

  ik_.init(kinematic_model_);
  visualizer_.init(kinematic_model_);

  current_state_.reset(new moveit::core::RobotState(kinematic_model_));
  current_state_->setToDefaultValues();

  walk_engine_.reset();

  // Publish the starting support state once, especially for odometry.
  // We always start with the same foot
  biped_interfaces::msg::Phase sup_state;
  sup_state.phase = biped_interfaces::msg::Phase::LEFT_STANCE;
  sup_state.header.stamp = node_->get_clock()->now();
  pub_support_->publish(sup_state);
}

void WalkNode::updateParams() {
  config_ = param_listener_.get_params();
  // Pass params to other components
  ik_.setConfig(config_.node.ik);
  walk_engine_.setConfig(config_.engine);
  // Phase reset can only work if one phase resetting method is active and this might have changed due to parameter
  // changes, but they are in a separate struct, so we have to check them manually
  walk_engine_.setPhaseRest(config_.node.phase_reset.effort.active || config_.node.phase_reset.foot_pressure.active ||
                            config_.node.phase_reset.imu.active);
  walk_engine_.setPauseDuration(config_.node.stability_stop.pause_duration);
}

void WalkNode::run() {
  // Get up to date parameters
  updateParams();

  double dt = getTimeDelta();
  // necessary as timer in simulation does not work correctly https://github.com/ros2/rclcpp/issues/465
  if (dt != 0.0) {
    if (robot_state_ == bitbots_msgs::msg::RobotControlState::FALLING ||
        robot_state_ == bitbots_msgs::msg::RobotControlState::GETTING_UP) {
      // the robot fell, we have to reset everything and do nothing else
      walk_engine_.reset();
      stabilizer_.reset();
    } else {
      // we don't want to walk, even if we have orders, if we are not in the right state
      /* Our robots will soon^TM be able to sit down and stand up autonomously, when sitting down the motors are
       * off but will turn on automatically which is why MOTOR_OFF is a valid walkable state. */
      // TODO Figure out a better way than having integration knowledge that HCM will play an animation to stand up
      current_request_.walkable_state = robot_state_ == bitbots_msgs::msg::RobotControlState::CONTROLLABLE ||
                                        robot_state_ == bitbots_msgs::msg::RobotControlState::WALKING ||
                                        robot_state_ == bitbots_msgs::msg::RobotControlState::MOTOR_OFF;

      // reset when we start walking, otherwise PID controller will use old I value
      if ((last_request_.linear_orders[0] == 0 && last_request_.linear_orders[1] == 0 &&
           last_request_.angular_z == 0) &&
          (current_request_.linear_orders[0] != 0 || current_request_.linear_orders[1] != 0 ||
           current_request_.angular_z != 0)) {
        stabilizer_.reset();
      }
      last_request_ = current_request_;

      // perform all the actual calculations
      bitbots_msgs::msg::JointCommand joint_goals = step(dt);

      // only publish goals if we are not idle
      if (walk_engine_.getState() != WalkState::IDLE) {
        pub_controller_command_->publish(joint_goals);

        // publish current support state
        biped_interfaces::msg::Phase support_state;
        if (walk_engine_.isDoubleSupport()) {
          support_state.phase = biped_interfaces::msg::Phase::DOUBLE_STANCE;
        } else if (walk_engine_.isLeftSupport()) {
          support_state.phase = biped_interfaces::msg::Phase::LEFT_STANCE;
        } else {
          support_state.phase = biped_interfaces::msg::Phase::RIGHT_STANCE;
        }
        // publish if foot changed
        if (current_support_foot_ != support_state.phase) {
          support_state.header.stamp = node_->get_clock()->now();
          pub_support_->publish(support_state);
          current_support_foot_ = support_state.phase;
        }
      }
    }

    // publish debug information
    if (config_.node.debug_active) {
      publish_debug();
    }
    // always publish odometry to not confuse odometry fuser
    pub_odometry_->publish(getOdometry());
  }
}

void WalkNode::publish_debug() { visualizer_.publishDebug(current_stabilized_response_, current_state_, motor_goals_); }

bitbots_msgs::msg::JointCommand WalkNode::step(double dt) {
  WalkRequest request(current_request_);

  // update walk engine response
  if (got_new_goals_) {
    got_new_goals_ = false;
    walk_engine_.setGoals(request);
  }
  checkPhaseRestAndReset();
  current_response_ = walk_engine_.update(dt);

  // only calculate joint goals from this if the engine is not idle
  current_response_.current_fused_roll = current_trunk_fused_roll_;
  current_response_.current_fused_pitch = current_trunk_fused_pitch_;

  bitbots_msgs::msg::JointCommand command;
  if (walk_engine_.getState() != WalkState::IDLE) {
    // get stabilized goals from stabilizer
    current_stabilized_response_ =
        stabilizer_.stabilize(current_response_, rclcpp::Duration::from_nanoseconds(1e9 * dt));

    // compute motor goals from IK
    motor_goals_ = ik_.calculate(current_stabilized_response_);

    command.header.stamp = node_->get_clock()->now();
    /*
     * Since our JointGoals type is a vector of strings
     *  combined with a vector of numbers (motor name -> target position)
     *  and bitbots_msgs::msg::JointCommand needs both vectors as well,
     *  we can just assign them
     */
    command.joint_names = motor_goals_.first;
    command.positions = motor_goals_.second;

    /* And because we are setting position goals and not movement goals, these vectors are set to -1.0*/
    std::vector<double> vels(motor_goals_.first.size(), -1.0);
    std::vector<double> accs(motor_goals_.first.size(), -1.0);
    std::vector<double> pwms(motor_goals_.first.size(), -1.0);
    command.velocities = vels;
    command.accelerations = accs;
    command.max_currents = pwms;
  }

  return command;
}

double WalkNode::getTimeDelta() {
  // compute time delta depended if we are currently in simulation or reality
  double dt;
  double current_ros_time = node_->get_clock()->now().seconds();
  dt = current_ros_time - last_ros_update_time_;
  last_ros_update_time_ = current_ros_time;

  // time is wrong when we run it for the first time
  if (first_run_) {
    first_run_ = false;
    dt = 0.0001;
  }
  return dt;
}

void WalkNode::reset() {
  walk_engine_.reset();
  stabilizer_.reset();
}

void WalkNode::reset(WalkState state, double phase, geometry_msgs::msg::Twist::SharedPtr cmd_vel, bool reset_odometry) {
  auto step = get_step_from_vel(cmd_vel);
  bool stop_walk = cmd_vel->angular.x < 0;
  walk_engine_.reset(state, phase, step, stop_walk, true, reset_odometry);
  stabilizer_.reset();
  cmdVelCb(cmd_vel);
}

bitbots_msgs::msg::JointCommand WalkNode::step(double dt, const geometry_msgs::msg::Twist::SharedPtr cmdvel_msg,
                                               const sensor_msgs::msg::Imu::SharedPtr imu_msg,
                                               const sensor_msgs::msg::JointState::SharedPtr jointstate_msg,
                                               const bitbots_msgs::msg::FootPressure::SharedPtr pressure_left,
                                               const bitbots_msgs::msg::FootPressure::SharedPtr pressure_right) {
  // method for python interface. take all messages as parameters instead of using ROS
  cmdVelCb(cmdvel_msg);
  imuCb(imu_msg);
  jointStateCb(jointstate_msg);
  pressureLeftCb(pressure_left);
  pressureRightCb(pressure_right);
  // we don't use external robot state
  current_request_.walkable_state = true;
  // update walk engine response
  bitbots_msgs::msg::JointCommand joint_goals = step(dt);
  return joint_goals;
}

bitbots_msgs::msg::JointCommand WalkNode::step_relative(
    double dt, const geometry_msgs::msg::Twist::SharedPtr step_msg, const sensor_msgs::msg::Imu::SharedPtr imu_msg,
    const sensor_msgs::msg::JointState::SharedPtr jointstate_msg,
    const bitbots_msgs::msg::FootPressure::SharedPtr pressure_left,
    const bitbots_msgs::msg::FootPressure::SharedPtr pressure_right) {
  // method for python interface. take all messages as parameters instead of using ROS
  // use length of next step instead of cmd_vel
  got_new_goals_ = true;
  current_request_.single_step = false;

  current_request_.linear_orders = {step_msg->linear.x, step_msg->linear.y, step_msg->linear.z};
  current_request_.angular_z = step_msg->angular.z;

  // special command to completely stop the walking
  current_request_.stop_walk = step_msg->angular.x != 0;

  imuCb(imu_msg);
  jointStateCb(jointstate_msg);
  pressureLeftCb(pressure_left);
  pressureRightCb(pressure_right);
  // we don't use external robot state
  current_request_.walkable_state = true;
  // update walk engine response
  bitbots_msgs::msg::JointCommand joint_goals = step(dt);
  return joint_goals;
}

geometry_msgs::msg::PoseArray WalkNode::step_open_loop(double dt,
                                                       const geometry_msgs::msg::Twist::SharedPtr cmdvel_msg) {
  cmdVelCb(cmdvel_msg);
  // get cartesian goals from open loop engine
  WalkRequest request(current_request_);
  walk_engine_.setGoals(request);
  current_response_ = walk_engine_.update(dt);

  // change goals from support foot based coordinate system to trunk based coordinate system
  tf2::Transform trunk_to_support_foot_goal = current_response_.support_foot_to_trunk.inverse();
  tf2::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal * current_response_.support_foot_to_flying_foot;
  geometry_msgs::msg::Pose left_foot_goal_msg;
  geometry_msgs::msg::Pose right_foot_goal_msg;
  // decide which foot is which
  if (current_response_.is_left_support_foot) {
    bitbots_quintic_walk::tf_pose_to_msg(trunk_to_support_foot_goal, left_foot_goal_msg);
    bitbots_quintic_walk::tf_pose_to_msg(trunk_to_flying_foot_goal, right_foot_goal_msg);
  } else {
    bitbots_quintic_walk::tf_pose_to_msg(trunk_to_support_foot_goal, right_foot_goal_msg);
    bitbots_quintic_walk::tf_pose_to_msg(trunk_to_flying_foot_goal, left_foot_goal_msg);
  }
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.poses = {left_foot_goal_msg, right_foot_goal_msg};
  return pose_array;
}

geometry_msgs::msg::Pose WalkNode::get_left_foot_pose() {
  moveit::core::RobotStatePtr goal_state = ik_.get_goal_state();
  geometry_msgs::msg::Pose pose;
  tf2::convert(goal_state->getGlobalLinkTransform("l_sole"), pose);
  return pose;
}

geometry_msgs::msg::Pose WalkNode::get_right_foot_pose() {
  moveit::core::RobotStatePtr goal_state = ik_.get_goal_state();
  geometry_msgs::msg::Pose pose;
  tf2::convert(goal_state->getGlobalLinkTransform("r_sole"), pose);
  return pose;
}

std::array<double, 4> WalkNode::get_step_from_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // We have to compute by dividing by step frequency which is a double step
  // factor 2 since the order distance is only for a single step, not double step
  double factor = (1.0 / (walk_engine_.getFreq())) / 2.0;
  // the sidewards movement only does one step per double step, since the second foot only goes back to the initial
  // pose therefore we need to multiply it by 2 furthermore, the engine does not really reach the correct goal
  // speed, dependent on the parameters
  std::array<double, 4> step = {msg->linear.x * factor * config_.node.x_speed_multiplier + config_.node.x_bias,
                                msg->linear.y * factor * 2 * config_.node.y_speed_multiplier + config_.node.y_bias,
                                msg->linear.z * factor,
                                msg->angular.z * factor * config_.node.yaw_speed_multiplier + config_.node.yaw_bias};

  std::array<double, 4> clamped_step = {
      // the orders should not extend beyond a maximal step size
      std::clamp(step[0], -config_.node.max_step_x, config_.node.max_step_x),
      std::clamp(step[1], -config_.node.max_step_y, config_.node.max_step_y),
      std::clamp(step[2], -config_.node.max_step_z, config_.node.max_step_z),
      std::max(std::min(step[3], config_.node.max_step_angular), config_.node.max_step_angular * -1)};

  // translational orders (x+y) should not exceed combined limit. scale if necessary
  if (config_.node.max_step_xy != 0) {
    double scaling_factor = sqrt(pow(clamped_step[0], 2) + pow(clamped_step[1], 2)) / config_.node.max_step_xy;
    for (int i = 0; i < 2; i++) {
      clamped_step[i] = clamped_step[i] / std::max(scaling_factor, 1.0);
    }
  }

  // warn user that speed was limited
  if (step != clamped_step) {
    RCLCPP_WARN(
        node_->get_logger(),
        "Commanded step 'x: %.2f y: %.2f z: %.2f angular: %.2f' exceeded limits, clamped to 'x: %.2f y: %.2f z: "
        "%.2f angular: %.2f'",
        step[0], step[1], step[2], step[3], clamped_step[0], clamped_step[1], clamped_step[2], clamped_step[3]);
  }

  return clamped_step;
}
void WalkNode::stepCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  current_request_.linear_orders = {msg->linear.x, msg->linear.y, msg->linear.z};
  current_request_.angular_z = msg->angular.z;
  current_request_.single_step = true;
  current_request_.stop_walk = true;
  got_new_goals_ = true;
}

void WalkNode::cmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg) {
  got_new_goals_ = true;
  current_request_.single_step = false;

  // we use only 3 values from the twist messages, as the robot is not capable of jumping or spinning around its
  // other axis.

  // the engine expects orders in [m] not [m/s]
  auto step = get_step_from_vel(msg);
  current_request_.linear_orders = {step[0], step[1], step[2]};
  current_request_.angular_z = step[3];

  // special command to completely stop the walking
  current_request_.stop_walk = msg->angular.x != 0;
}

void WalkNode::imuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // the incoming geometry_msgs::msg::Quaternion is transformed to a tf2::Quaternion
  tf2::Quaternion quat;
  tf2::convert(msg->orientation, quat);
  // the tf2::Quaternion has a method to access roll pitch and yaw
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  Eigen::Quaterniond imu_orientation_eigen = Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
  rot_conv::FusedAngles imu_fused = rot_conv::FusedFromQuat(imu_orientation_eigen);

  current_trunk_fused_pitch_ = imu_fused.fusedPitch;
  current_trunk_fused_roll_ = imu_fused.fusedRoll;

  // get angular velocities
  double roll_vel = msg->angular_velocity.x;
  double pitch_vel = msg->angular_velocity.y;

  // Calculate ema (exponential moving average) of y (sideways) acceleration in a time-independet manner
  if (last_imu_measurement_time_) {
    auto time_delta = rclcpp::Time(msg->header.stamp) - last_imu_measurement_time_.value();
    double exponent_in_s = -time_delta.nanoseconds() / (config_.node.imu_y_acc_tau * 1e9);
    double alpha = 1 - std::exp(exponent_in_s);

    imu_y_acc = msg->linear_acceleration.y * alpha + imu_y_acc * (1 - alpha);
  } else {
    imu_y_acc = msg->linear_acceleration.y;
  }

  if (config_.node.stability_stop.imu.active) {
    // compute the pitch offset to the currently wanted pitch of the engine
    double wanted_pitch = walk_engine_.getWantedTrunkPitch();

    // Get the sub struct of the imu stability stop config
    auto params = config_.node.stability_stop.imu;

    // Check if we have to stop the walk
    double pitch_delta = pitch - wanted_pitch;
    if (abs(roll) > params.roll.threshold or abs(pitch_delta) > params.pitch.threshold or
        abs(pitch_vel) > params.pitch.vel_threshold or abs(roll_vel) > params.roll.vel_threshold) {
      walk_engine_.requestPause();
      if (abs(roll) > params.roll.threshold) {
        RCLCPP_WARN(node_->get_logger(), "imu roll angle stop");
      } else if (abs(pitch_delta) > params.pitch.threshold) {
        RCLCPP_WARN(node_->get_logger(), "imu pitch angle stop");
      } else if (abs(pitch_vel) > params.pitch.vel_threshold) {
        RCLCPP_WARN(node_->get_logger(), "imu roll vel stop");
      } else {
        RCLCPP_WARN(node_->get_logger(), "imu pitch vel stop");
      }
    }
  }
  // Store the timestamp
  last_imu_measurement_time_ = msg->header.stamp;
}

void WalkNode::pressureLeftCb(const bitbots_msgs::msg::FootPressure::SharedPtr msg) {
  // only check if this foot is not the current support foot
  if (!walk_engine_.isLeftSupport()) {
    current_fly_pressure_ = msg->left_back + msg->left_front + msg->right_back + msg->right_front;
  }
}

void WalkNode::pressureRightCb(const bitbots_msgs::msg::FootPressure::SharedPtr msg) {
  // only check if this foot is not the current support foot
  if (walk_engine_.isLeftSupport()) {
    current_fly_pressure_ = msg->left_back + msg->left_front + msg->right_back + msg->right_front;
  }
}

void WalkNode::checkPhaseRestAndReset() {
  /**
   * This method checks if the foot made contact to the ground and ends the step earlier by resetting the phase
   ("phase reset") or lets the robot rest until it makes ground contact ("phase rest").
   */
  double phase = walk_engine_.getPhase();
  // Check if we are in the correct point of the phase to do a phase reset
  // Phase has to be far enough (almost at end of step) so that the foot has already lifted from the ground
  // otherwise we will always do phase reset in the beginning of the step
  double min_phase_single_step = config_.node.phase_reset.min_phase / 2;
  if ((phase > min_phase_single_step and phase < 0.5) or (phase > 0.5 + min_phase_single_step)) {
    // Check if one of our phase reset conditions is met
    if ((config_.node.phase_reset.foot_pressure.active and
         current_fly_pressure_ > config_.node.phase_reset.foot_pressure.ground_min_pressure) or
        (config_.node.phase_reset.effort.active and
         current_fly_effort_ > config_.node.phase_reset.effort.joint_min_effort) or
        (config_.node.phase_reset.imu.active and
         std::abs(imu_y_acc) < config_.node.phase_reset.imu.y_acceleration_threshold)) {
      // Manually end the step
      // This may and a step earlier than the engine would do it
      // But it can also lengthen the step, as the engine will not end the step by itself if a phase reset is active
      walk_engine_.endStep();
    }
  }
}

void WalkNode::robotStateCb(const bitbots_msgs::msg::RobotControlState::SharedPtr msg) { robot_state_ = msg->state; }

void WalkNode::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::vector<std::string> names = msg->name;
  std::vector<double> goals = msg->position;
  for (size_t i = 0; i < names.size(); i++) {
    // besides its name, this method only changes a single joint position...
    current_state_->setJointPositions(names[i], &goals[i]);
  }

  // compute the effort that is currently on the flying leg to check if it has ground contact
  // only if we have the necessary data
  if (msg->effort.size() == msg->name.size()) {
    double effort_sum = 0;
    const std::vector<std::string>& fly_joint_names =
        (walk_engine_.isLeftSupport()) ? ik_.getRightLegJointNames() : ik_.getLeftLegJointNames();
    for (size_t i = 0; i < names.size(); i++) {
      // add effort on this joint to sum, if it is part of the flying leg
      if (std::find(fly_joint_names.begin(), fly_joint_names.end(), names[i]) != fly_joint_names.end()) {
        effort_sum = effort_sum + abs(msg->effort[i]);
      }
    }
    current_fly_effort_ = effort_sum;
  } else {
    RCLCPP_WARN_ONCE(node_->get_logger(),
                     "Joint states have no effort information. Phase reset based on this will not work.");
  }
}

void WalkNode::kickCb(const std_msgs::msg::Bool::SharedPtr msg) { walk_engine_.requestKick(msg->data); }

nav_msgs::msg::Odometry WalkNode::getOdometry() {
  // odometry to trunk is transform to support foot * transform from support to trunk
  tf2::Transform support_foot_tf;
  if (walk_engine_.isLeftSupport()) {
    support_foot_tf = walk_engine_.getLeft();
  } else {
    support_foot_tf = walk_engine_.getRight();
  }

  tf2::Transform odom_to_trunk = support_foot_tf * current_response_.support_foot_to_trunk;
  tf2::Vector3 pos = odom_to_trunk.getOrigin();
  geometry_msgs::msg::Quaternion quat_msg;
  tf2::Quaternion quat_normalized = odom_to_trunk.getRotation().normalize();
  quat_msg.x = quat_normalized.x();
  quat_msg.y = quat_normalized.y();
  quat_msg.z = quat_normalized.z();
  quat_msg.w = quat_normalized.w();

  rclcpp::Time current_time = node_->get_clock()->now();

  // send the odometry as message
  odom_msg_.header.stamp = current_time;
  odom_msg_.header.frame_id = config_.node.tf.odom_frame;
  odom_msg_.child_frame_id = config_.node.tf.base_link_frame;
  odom_msg_.pose.pose.position.x = pos[0];
  odom_msg_.pose.pose.position.y = pos[1];
  odom_msg_.pose.pose.position.z = pos[2];
  odom_msg_.pose.pose.orientation = quat_msg;

  geometry_msgs::msg::Twist twist;
  twist.linear.x = current_request_.linear_orders[0] * walk_engine_.getFreq() * 2 / config_.node.x_speed_multiplier;
  twist.linear.y = current_request_.linear_orders[1] * walk_engine_.getFreq() / config_.node.y_speed_multiplier;
  twist.linear.z = current_request_.linear_orders[2] * walk_engine_.getFreq() * 2;
  twist.angular.z = current_request_.angular_z * walk_engine_.getFreq() * 2 / config_.node.yaw_speed_multiplier;

  odom_msg_.twist.twist = twist;
  return odom_msg_;
}

void WalkNode::initializeEngine() { walk_engine_.reset(); }

WalkEngine* WalkNode::getEngine() { return &walk_engine_; }

WalkIK* WalkNode::getIk() { return &ik_; }

moveit::core::RobotModelPtr* WalkNode::get_kinematic_model() { return &kinematic_model_; }

double WalkNode::getTimerFreq() { return config_.node.engine_freq; }

}  // namespace bitbots_quintic_walk

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // init node
  auto node = std::make_shared<rclcpp::Node>("walking");

  // Create an executor that will be used to execute callbacks for our node.
  rclcpp::experimental::executors::EventsExecutor exec;

  // Init node part of the walking
  bitbots_quintic_walk::WalkNode walk_node(node);
  walk_node.initializeEngine();

  // Create timer that calls the run method of the walking regularly
  rclcpp::Duration timer_duration = rclcpp::Duration::from_seconds(1.0 / walk_node.getTimerFreq());
  rclcpp::TimerBase::SharedPtr timer =
      rclcpp::create_timer(node, node->get_clock(), timer_duration, [&walk_node]() -> void { walk_node.run(); });

  // Run the executor so everything is executed
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
}
