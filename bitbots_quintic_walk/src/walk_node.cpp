#define M_TAU M_PI * 2

#include "bitbots_quintic_walk/walk_node.h"

#include <memory>
#include <iostream>
using std::placeholders::_1;
using namespace std::chrono_literals;

namespace bitbots_quintic_walk {

WalkNode::WalkNode(const std::string ns) :
    Node("walking", rclcpp::NodeOptions().allow_undeclared_parameters(true)),
    walk_engine_(SharedPtr(this)),
    robot_model_loader_(SharedPtr(this), "robot_description", false),
    stabilizer_(),
    visualizer_(SharedPtr(this)) {

  // get all kinematics parameters from the move_group node
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/move_group");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  rcl_interfaces::msg::ListParametersResult
      parameter_list = parameters_client->list_parameters({"robot_description_kinematics"}, 10);
  auto parameters = parameters_client->get_parameters(parameter_list.names);
  // set the parameters to our node
  this->set_parameters(parameters);

  this->get_parameter("base_link_frame", base_link_frame_);
  this->get_parameter("r_sole_frame", r_sole_frame_);
  this->get_parameter("l_sole_frame", l_sole_frame_);
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->get_parameter("odom_frame", odom_frame_);

  // init variables
  robot_state_ = humanoid_league_msgs::msg::RobotControlState::CONTROLLABLE;
  current_request_.linear_orders = {0, 0, 0};
  current_request_.angular_z = 0;
  current_request_.stop_walk = true;
  current_trunk_fused_pitch_ = 0;
  current_trunk_fused_roll_ = 0;
  current_fly_pressure_ = 0;
  current_fly_effort_ = 0;

  x_speed_multiplier_ = 1;
  y_speed_multiplier_ = 1;
  yaw_speed_multiplier_ = 1;


  // read config
  this->declare_parameter<double>("node.engine_freq", 100.0);
  this->declare_parameter<bool>("node.debug_active", false);
  this->declare_parameter<int>("node.odom_pub_factor", 0);
  this->declare_parameter<double>("node.ik_timeout", 0);
  this->declare_parameter<double>("node.ground_min_pressure", 0);
  this->declare_parameter<double>("node.phase_reset_phase", 0);
  this->declare_parameter<double>("node.joint_min_effort", 0);
  this->declare_parameter<double>("node.pause_duration", 0);
  this->declare_parameter<bool>("node.imu_active", false);
  this->declare_parameter<double>("node.imu_pitch_threshold", 0);
  this->declare_parameter<double>("node.imu_roll_threshold", 0);
  this->declare_parameter<double>("node.imu_pitch_vel_threshold", 0);
  this->declare_parameter<double>("node.imu_roll_vel_threshold", 0);
  this->declare_parameter<double>("node.max_step_x", 0);
  this->declare_parameter<double>("node.max_step_y", 0);
  this->declare_parameter<double>("node.max_step_xy", 0);
  this->declare_parameter<double>("node.max_step_z", 0);
  this->declare_parameter<double>("node.max_step_angular", 0);
  this->declare_parameter<double>("node.x_speed_multiplier", 0);
  this->declare_parameter<double>("node.y_speed_multiplier", 0);
  this->declare_parameter<double>("node.yaw_speed_multiplier", 0);

  // read config values one time during start
  this->get_parameter("node.engine_freq", engine_frequency_);
  this->get_parameter("node.debug_active", debug_active_);
  this->get_parameter("node.odom_pub_factor", odom_pub_factor_);
  double timeout;
  this->get_parameter("node.ik_timeout", timeout);
  ik_.setIKTimeout(timeout);
  this->get_parameter("node.ground_min_pressure", ground_min_pressure_);
  this->get_parameter("node.phase_reset_phase", phase_reset_phase_);
  this->get_parameter("node.joint_min_effort", joint_min_effort_);
  double pause_duration;
  this->get_parameter("node.pause_duration", pause_duration);
  walk_engine_.setPauseDuration(pause_duration);
  this->get_parameter("node.imu_active", imu_active_);
  this->get_parameter("node.imu_pitch_threshold", imu_pitch_threshold_);
  this->get_parameter("node.imu_roll_threshold", imu_roll_threshold_);
  this->get_parameter("node.imu_pitch_vel_threshold", imu_pitch_vel_threshold_);
  this->get_parameter("node.imu_roll_vel_threshold", imu_roll_vel_threshold_);
  this->get_parameter("node.max_step_x", max_step_linear_[0]);
  this->get_parameter("node.max_step_y", max_step_linear_[1]);
  this->get_parameter("node.max_step_z", max_step_linear_[2]);
  this->get_parameter("node.max_step_xy", max_step_xy_);
  this->get_parameter("node.max_step_angular", max_step_angular_);
  this->get_parameter("node.x_speed_multiplier", x_speed_multiplier_);
  this->get_parameter("node.y_speed_multiplier", y_speed_multiplier_);
  this->get_parameter("node.yaw_speed_multiplier", yaw_speed_multiplier_);
  if (x_speed_multiplier_ == 0) {
    x_speed_multiplier_ = 1;
  }
  if (y_speed_multiplier_ == 0) {
    y_speed_multiplier_ = 1;
  }
  if (yaw_speed_multiplier_ == 0) {
    yaw_speed_multiplier_ = 1;
  }

  /* init publisher and subscriber */
  pub_controller_command_ = this->create_publisher<bitbots_msgs::msg::JointCommand>("walking_motor_goals", 1);
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("walk_engine_odometry", 1);
  pub_support_ = this->create_publisher<bitbots_msgs::msg::SupportState>("walk_support_state", 1);
  step_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("step", 1, std::bind(&WalkNode::stepCb, this, _1));
  cmd_vel_sub_ =
      this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&WalkNode::cmdVelCb, this, _1));
  robot_state_sub_ = this->create_subscription<humanoid_league_msgs::msg::RobotControlState>("robot_state",
                                                                                             1,
                                                                                             std::bind(&WalkNode::robotStateCb,
                                                                                                       this,
                                                                                                       _1));
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states",
                                                                             1,
                                                                             std::bind(&WalkNode::jointStateCb,
                                                                                       this,
                                                                                       _1));
  kick_sub_ = this->create_subscription<std_msgs::msg::Bool>("kick", 1, std::bind(&WalkNode::kickCb, this, _1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, std::bind(&WalkNode::imuCb, this, _1));
  pressure_sub_left_ = this->create_subscription<bitbots_msgs::msg::FootPressure>("foot_pressure_left/filtered",
                                                                                  1,
                                                                                  std::bind(&WalkNode::pressureLeftCb,
                                                                                            this,
                                                                                            _1));
  pressure_sub_right_ = this->create_subscription<bitbots_msgs::msg::FootPressure>("foot_pressure_right/filtered",
                                                                                   1,
                                                                                   std::bind(&WalkNode::pressureRightCb,
                                                                                             this,
                                                                                             _1));

  //load MoveIt! model
  robot_model_loader_.loadKinematicsSolvers();
  kinematic_model_ = robot_model_loader_.getModel();
  if (!kinematic_model_) {
    RCLCPP_FATAL(this->get_logger(), "No robot model loaded, killing quintic walk.");
    exit(1);
  }
  //stabilizer_.setRobotModel(kinematic_model_);
  ik_.init(kinematic_model_);
  visualizer_.init(kinematic_model_);

  current_state_.reset(new moveit::core::RobotState(kinematic_model_));
  current_state_->setToDefaultValues();

  first_run_ = true;

  callback_handle_ = this->add_on_set_parameters_callback(std::bind(&WalkNode::onSetParameters, this, _1));
}

void WalkNode::run() {
  int odom_counter = 0;
  walk_engine_.reset();
  WalkResponse response;
  // publish the starting support state once, especially for odometry. we always start with the same foot
  bitbots_msgs::msg::SupportState sup_state;
  sup_state.state = bitbots_msgs::msg::SupportState::LEFT;
  sup_state.header.stamp = this->get_clock()->now();
  pub_support_->publish(sup_state);

  rclcpp::Rate loop_rate(1000);
  double dt;
  WalkRequest last_request;
  while (rclcpp::ok()) {
    rclcpp::spin_some(this->get_node_base_interface());
    if (loop_rate.sleep()) {
      dt = getTimeDelta();

      if (robot_state_ == humanoid_league_msgs::msg::RobotControlState::FALLING
          || robot_state_ == humanoid_league_msgs::msg::RobotControlState::GETTING_UP) {
        // the robot fell, we have to reset everything and do nothing else
        walk_engine_.reset();
        stabilizer_.reset();
      } else {
        // we don't want to walk, even if we have orders, if we are not in the right state
        /* Our robots will soon^TM be able to sit down and stand up autonomously, when sitting down the motors are
         * off but will turn on automatically which is why MOTOR_OFF is a valid walkable state. */
        // TODO Figure out a better way than having integration knowledge that HCM will play an animation to stand up
        current_request_.walkable_state = robot_state_ == humanoid_league_msgs::msg::RobotControlState::CONTROLLABLE ||
            robot_state_ == humanoid_league_msgs::msg::RobotControlState::WALKING ||
            robot_state_ == humanoid_league_msgs::msg::RobotControlState::MOTOR_OFF;

        // reset when we start walking, otherwise PID controller will use old I value
        if ((last_request.linear_orders[0] == 0 && last_request.linear_orders[1] == 0 && last_request.angular_z == 0)
            &&
                (current_request_.linear_orders[0] != 0 || current_request_.linear_orders[1] != 0
                    || current_request_.angular_z != 0)) {
          stabilizer_.reset();
        }
        last_request = current_request_;

        // perform all the actual calculations
        bitbots_msgs::msg::JointCommand joint_goals = step(dt);

        // only publish goals if we are not idle
        if (walk_engine_.getState() != WalkState::IDLE) {
          pub_controller_command_->publish(joint_goals);

          // publish current support state
          bitbots_msgs::msg::SupportState support_state;
          if (walk_engine_.isDoubleSupport()) {
            support_state.state = bitbots_msgs::msg::SupportState::DOUBLE;
          } else if (walk_engine_.isLeftSupport()) {
            support_state.state = bitbots_msgs::msg::SupportState::LEFT;
          } else {
            support_state.state = bitbots_msgs::msg::SupportState::RIGHT;
          }
          // publish if foot changed
          if (current_support_foot_ != support_state.state) {
            pub_support_->publish(support_state);
            current_support_foot_ = support_state.state;
          }

          // publish debug information
          if (debug_active_) {
            visualizer_.publishIKDebug(current_stabilized_response_, current_state_, motor_goals_);
            visualizer_.publishWalkMarkers(current_stabilized_response_);
            visualizer_.publishEngineDebug(current_response_);
          }
        }
      }
      // always publish odometry to not confuse odometry fuser
      odom_counter++;
      if (odom_counter > odom_pub_factor_) {
        pub_odometry_->publish(getOdometry());
        odom_counter = 0;
      }
    } else {
      usleep(1);
    }
  }
}

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

  // get stabilized goals from stabilizer
  current_stabilized_response_ = stabilizer_.stabilize(current_response_, rclcpp::Duration::from_nanoseconds(1e9 * dt));

  // compute motor goals from IK
  motor_goals_ = ik_.calculate(current_stabilized_response_);

  // change to joint command message type
  bitbots_msgs::msg::JointCommand command;
  command.header.stamp = this->get_clock()->now();
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

  return command;
}

double WalkNode::getTimeDelta() {
  // compute time delta depended if we are currently in simulation or reality
  double dt;
  double current_ros_time = this->get_clock()->now().seconds();
  dt = current_ros_time - last_ros_update_time_;
  if (dt == 0) {
    RCLCPP_WARN(this->get_logger(), "dt was 0");
    dt = 0.001;
  }
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
  std::vector<double> step = get_step_from_vel(cmd_vel);
  bool stop_walk = cmd_vel->angular.x < 0;
  walk_engine_.reset(state, phase, step, stop_walk, true, reset_odometry);
  stabilizer_.reset();
  cmdVelCb(cmd_vel);
}

bitbots_msgs::msg::JointCommand WalkNode::step(double dt,
                                               const geometry_msgs::msg::Twist::SharedPtr cmdvel_msg,
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

bitbots_msgs::msg::JointCommand WalkNode::step_relative(double dt,
                                               const geometry_msgs::msg::Twist::SharedPtr step_msg,
                                               const sensor_msgs::msg::Imu::SharedPtr imu_msg,
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

std::vector<double> WalkNode::get_step_from_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // We have to compute by dividing by step frequency which is a double step
  // factor 2 since the order distance is only for a single step, not double step
  double factor = (1.0 / (walk_engine_.getFreq())) / 2.0;
  // the sidewards movement only does one step per double step, since the second foot only goes back to the initial pose
  // therefore we need to multiply it by 2
  // furthermore, the engine does not really reach the correct goal speed, dependent on the parameters
  std::vector<double> step = {msg->linear.x * factor * x_speed_multiplier_,
                              msg->linear.y * factor * 2 * y_speed_multiplier_,
                              msg->linear.z * factor,
                              msg->angular.z * factor * yaw_speed_multiplier_};

  // the orders should not extend beyond a maximal step size
  for (int i = 0; i < 3; i++) {
    step[i] = std::max(std::min(step[i], max_step_linear_[i]), max_step_linear_[i] * -1);
  }
  step[3] = std::max(std::min(step[3], max_step_angular_), max_step_angular_ * -1);

  // translational orders (x+y) should not exceed combined limit. scale if necessary
  if (max_step_xy_ != 0) {
    double scaling_factor = sqrt(pow(step[0], 2) + pow(step[1], 2)) / max_step_xy_;
    for (int i = 0; i < 2; i++) {
      step[i] = step[i] / std::max(scaling_factor, 1.0);
    }
  }

  // warn user that speed was limited
  if (msg->linear.x * factor * x_speed_multiplier_ != step[0] ||
      msg->linear.y * factor * y_speed_multiplier_ != step[1] / 2 ||
      msg->linear.z * factor != step[2] ||
      msg->angular.z * factor * yaw_speed_multiplier_ != step[3]) {
    RCLCPP_WARN(this->get_logger(),
                "Speed command was x: %.2f y: %.2f z: %.2f angular: %.2f xy: %.2f but maximum is x: %.2f y: %.2f z: %.2f angular: %.2f xy: %.2f",
                msg->linear.x,
                msg->linear.y,
                msg->linear.z,
                msg->angular.z,
                msg->linear.x + msg->linear.y,
                max_step_linear_[0] / factor,
                max_step_linear_[1] / factor / 2,
                max_step_linear_[2] / factor,
                max_step_angular_ / factor,
                max_step_xy_ / factor);
  }

  return step;
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
  std::vector<double> step = get_step_from_vel(msg);
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
  roll_vel_ = msg->angular_velocity.x;
  pitch_vel_ = msg->angular_velocity.y;

  if (imu_active_) {
    // compute the pitch offset to the currently wanted pitch of the engine
    double wanted_pitch = walk_engine_.getWantedTrunkPitch();

    double pitch_delta = pitch - wanted_pitch;
    if (abs(roll) > imu_roll_threshold_ || abs(pitch_delta) > imu_pitch_threshold_ ||
        abs(pitch_vel_) > imu_pitch_vel_threshold_ || abs(roll_vel_) > imu_roll_vel_threshold_) {
      walk_engine_.requestPause();
      if (abs(roll) > imu_roll_threshold_) {
        RCLCPP_WARN(this->get_logger(), "imu roll angle stop");
      } else if (abs(pitch_delta) > imu_pitch_threshold_) {
        RCLCPP_WARN(this->get_logger(), "imu pitch angle stop");
      } else if (abs(pitch_vel_) > imu_pitch_vel_threshold_) {
        RCLCPP_WARN(this->get_logger(), "imu roll vel stop");
      } else {
        RCLCPP_WARN(this->get_logger(), "imu pitch vel stop");
      }
    }
  }
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
   * This method checks if the foot made contact to the ground and ends the step earlier by resetting the phase ("phase reset")
     or lets the robot rest until it makes ground contact ("phase rest").
   */
  // phase has to be far enough (almost at end of step) so that the foot has already lifted from the ground
  // otherwise we will always do phase reset in the beginning of the step
  double phase = walk_engine_.getPhase();
  double phase_reset_phase = walk_engine_.getPhaseResetPhase();

  if ((phase > phase_reset_phase && phase < 0.5) || (phase > 0.5 + phase_reset_phase)) {
    // check if we want to perform a phase reset
    if (pressure_phase_reset_active_ && current_fly_pressure_ > ground_min_pressure_) {
      // reset phase by using pressure sensors
      walk_engine_.endStep();
    } else if (effort_phase_reset_active_ && current_fly_effort_ > joint_min_effort_) {
      // reset phase by using joint efforts
      walk_engine_.endStep();
    }
  }
}

void WalkNode::robotStateCb(const humanoid_league_msgs::msg::RobotControlState::SharedPtr msg) {
  robot_state_ = msg->state;
}

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
    const std::vector<std::string>
        &fly_joint_names = (walk_engine_.isLeftSupport()) ? ik_.getRightLegJointNames() : ik_.getLeftLegJointNames();
    for (size_t i = 0; i < names.size(); i++) {
      // add effort on this joint to sum, if it is part of the flying leg
      if (std::find(fly_joint_names.begin(), fly_joint_names.end(), names[i]) != fly_joint_names.end()) {
        effort_sum = effort_sum + abs(msg->effort[i]);
      }
    }
    current_fly_effort_ = effort_sum;
  } else {
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "Joint states have no effort information. Phase reset based on this will not work.");
  }
}

void WalkNode::kickCb(const std_msgs::msg::Bool::SharedPtr msg) {
  walk_engine_.requestKick(msg->data);
}

rcl_interfaces::msg::SetParametersResult WalkNode::onSetParameters(const std::vector<rclcpp::Parameter> &parameters) {
  for (const auto &parameter: parameters) {
    if (parameter.get_name() == "node.ik_timeout") {
      ik_.setIKTimeout(parameter.as_double());
    } else if (parameter.get_name() == "node.debug_active") {
      debug_active_ = parameter.as_bool();
    } else if (parameter.get_name() == "node.engine_freq") {
      engine_frequency_ = parameter.as_double();
    } else if (parameter.get_name() == "node.odom_pub_factor") {
      odom_pub_factor_ = parameter.as_int();
    } else if (parameter.get_name() == "node.max_step_x") {
      max_step_linear_[0] = parameter.as_double();
    } else if (parameter.get_name() == "node.max_step_y") {
      max_step_linear_[1] = parameter.as_double();
    } else if (parameter.get_name() == "node.max_step_z") {
      max_step_linear_[2] = parameter.as_double();
    } else if (parameter.get_name() == "node.max_step_angular") {
      max_step_angular_ = parameter.as_double();
    } else if (parameter.get_name() == "node.max_step_xy") {
      max_step_xy_ = parameter.as_double();
    } else if (parameter.get_name() == "node.x_speed_multiplier") {
      x_speed_multiplier_ = parameter.as_double();
    } else if (parameter.get_name() == "node.y_speed_multiplier") {
      y_speed_multiplier_ = parameter.as_double();
    } else if (parameter.get_name() == "node.yaw_speed_multiplier") {
      yaw_speed_multiplier_ = parameter.as_double();
    } else if (parameter.get_name() == "node.imu_active") {
      imu_active_ = parameter.as_bool();
    } else if (parameter.get_name() == "node.imu_pitch_threshold") {
      imu_pitch_threshold_ = parameter.as_double();
    } else if (parameter.get_name() == "node.imu_roll_threshold") {
      imu_roll_threshold_ = parameter.as_double();
    } else if (parameter.get_name() == "node.imu_pitch_vel_threshold") {
      imu_pitch_vel_threshold_ = parameter.as_double();
    } else if (parameter.get_name() == "node.imu_roll_vel_threshold") {
      imu_roll_vel_threshold_ = parameter.as_double();
    } else if (parameter.get_name() == "node.phase_reset_active") {
      phase_reset_active_ = parameter.as_bool();
    } else if (parameter.get_name() == "node.pressure_phase_reset_active") {
      pressure_phase_reset_active_ = parameter.as_bool();
    } else if (parameter.get_name() == "node.effort_phase_reset_active") {
      effort_phase_reset_active_ = parameter.as_bool();
    } else if (parameter.get_name() == "node.ground_min_pressure") {
      ground_min_pressure_ = parameter.as_double();
    } else if (parameter.get_name() == "node.joint_min_effort") {
      joint_min_effort_ = parameter.as_double();
    } else if (parameter.get_name() == "node.phase_reset_phase") {
      phase_reset_phase_ = parameter.as_double();
    } else if (parameter.get_name() == "node.pause_duration") {
      walk_engine_.setPauseDuration(parameter.as_double());
    } else {
      if (!walk_engine_.onSetParameters(parameter)) {
        RCLCPP_WARN(this->get_logger(), "Unknown parameter: %s", parameter.get_name().c_str());
      }
    }
  }

  if (x_speed_multiplier_ == 0 || y_speed_multiplier_ == 0 || yaw_speed_multiplier_ == 0) {
    RCLCPP_WARN(this->get_logger(), "some speed multipliers in walking are 0. check your config!");
  }

  // phase rest can only work if one phase resetting method is active
  if (effort_phase_reset_active_ || pressure_phase_reset_active_) {
    walk_engine_.setPhaseRest(phase_reset_active_);
  } else {
    walk_engine_.setPhaseRest(false);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

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

  rclcpp::Time current_time = this->get_clock()->now();

  // send the odometry as message
  odom_msg_.header.stamp = current_time;
  odom_msg_.header.frame_id = odom_frame_;
  odom_msg_.child_frame_id = base_link_frame_;
  odom_msg_.pose.pose.position.x = pos[0];
  odom_msg_.pose.pose.position.y = pos[1];
  odom_msg_.pose.pose.position.z = pos[2];
  odom_msg_.pose.pose.orientation = quat_msg;

  geometry_msgs::msg::Twist twist;
  twist.linear.x = current_request_.linear_orders[0] * walk_engine_.getFreq() * 2 / x_speed_multiplier_;
  twist.linear.y = current_request_.linear_orders[1] * walk_engine_.getFreq() / y_speed_multiplier_;
  twist.linear.z = current_request_.linear_orders[2] * walk_engine_.getFreq() * 2;
  twist.angular.z = current_request_.angular_z * walk_engine_.getFreq() * 2 / yaw_speed_multiplier_;

  odom_msg_.twist.twist = twist;
  return odom_msg_;
}

void WalkNode::initializeEngine() {
  walk_engine_.reset();
}

WalkEngine *WalkNode::getEngine() {
  return &walk_engine_;
}

} // namespace bitbots_quintic_walk

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // init node
  bitbots_quintic_walk::WalkNode node("");

  // run the node
  node.initializeEngine();
  node.run();
}