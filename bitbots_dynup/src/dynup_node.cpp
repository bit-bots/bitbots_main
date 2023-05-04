#include "bitbots_dynup/dynup_node.h"

namespace bitbots_dynup {
  using namespace std::chrono_literals;


  DynupNode::DynupNode(const std::string ns, std::vector<rclcpp::Parameter> parameters) :
    Node(ns + "dynup", rclcpp::NodeOptions().allow_undeclared_parameters(true).parameter_overrides(parameters).automatically_declare_parameters_from_overrides(true)),
    engine_(SharedPtr(this)),
    stabilizer_(ns),
    visualizer_("debug/dynup", SharedPtr(this)),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(this->get_clock())){
  // get all kinematics parameters from the move_group node if they are not set manually via constructor
  std::string check_kinematic_parameters;
  if (!this->get_parameter("robot_description_kinematics.LeftLeg.kinematics_solver", check_kinematic_parameters)) {
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/move_group");
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10*1e9, "Can't copy parameters from move_group node. Service not available, waiting again...");
    }
    rcl_interfaces::msg::ListParametersResult
      parameter_list = parameters_client->list_parameters({"robot_description_kinematics"}, 10);
    auto copied_parameters = parameters_client->get_parameters(parameter_list.names);
    // set the parameters to our node
    this->set_parameters(copied_parameters);
  }

  base_link_frame_ = this->get_parameter("base_link_frame").get_value<std::string>();
  r_sole_frame_ = this->get_parameter("r_sole_frame").get_value<std::string>();
  l_sole_frame_ = this->get_parameter("l_sole_frame").get_value<std::string>();
  r_wrist_frame_ = this->get_parameter("r_wrist_frame").get_value<std::string>();
  l_wrist_frame_ = this->get_parameter("l_wrist_frame").get_value<std::string>();

  param_names_ = {
    "engine_rate",
    "arm_extended_length",
    "foot_distance",
    "hand_walkready_pitch",
    "hand_walkready_height",
    "trunk_height",
    "trunk_pitch",
    "trunk_x_final",
    "time_walkready",
    "arms_angle_back",
    "arm_side_offset_back",
    "com_shift_1",
    "com_shift_2",
    "foot_angle",
    "hands_behind_back_x",
    "hands_behind_back_z",
    "leg_min_length_back",
    "time_foot_ground_back",
    "time_full_squat_hands",
    "time_full_squat_legs",
    "time_legs_close",
    "trunk_height_back",
    "trunk_overshoot_angle_back",
    "wait_in_squat_back",
    "arm_side_offset_front",
    "hands_pitch",
    "leg_min_length_front",
    "max_leg_angle",
    "time_foot_close",
    "time_foot_ground_front",
    "time_hands_front",
    "time_hands_rotate",
    "time_hands_side",
    "time_to_squat",
    "time_torso_45",
    "trunk_overshoot_angle_front",
    "trunk_x_front",
    "wait_in_squat_front",
    "rise_time",
    "descend_time",
    "stabilizing",
    "minimal_displacement",
    "stable_threshold",
    "stable_duration",
    "stabilization_timeout",
    "spline_smoothness",
    "display_debug",
    "pid_trunk_roll.p",
    "pid_trunk_roll.i",
    "pid_trunk_roll.d",
    "pid_trunk_roll.i_clamp",
    "pid_trunk_roll.i_clamp_min",
    "pid_trunk_roll.i_clamp_max",
    "pid_trunk_roll.antiwindup",
    "pid_trunk_roll.publish_state",
    "pid_trunk_pitch.p",
    "pid_trunk_pitch.i",
    "pid_trunk_pitch.d",
    "pid_trunk_pitch.i_clamp",
    "pid_trunk_pitch.i_clamp_min",
    "pid_trunk_pitch.i_clamp_max",
    "pid_trunk_pitch.antiwindup",
    "pid_trunk_pitch.publish_state"};

  // load params once
  const std::vector<rclcpp::Parameter> params;
  onSetParameters(params);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(SharedPtr(this), "robot_description", true);
  kinematic_model_ = robot_model_loader_->getModel();
  if (!kinematic_model_) {
      RCLCPP_FATAL(this->get_logger(), "No robot model loaded, killing dynup.");
      exit(1);
  }
  moveit::core::RobotStatePtr init_state;
  init_state.reset(new moveit::core::RobotState(kinematic_model_));
  // set elbows to make arms straight, in a stupid way since moveit is annoying
  std::vector<std::string> names_vec = {"LElbow", "RElbow"};
  std::vector<double> pos_vec = {-M_PI / 2, M_PI / 2};
  init_state->setJointPositions(names_vec[0], &pos_vec[0]);
  init_state->setJointPositions(names_vec[1], &pos_vec[1]);
  init_state->updateLinkTransforms();
  // get shoulder and wrist pose
  geometry_msgs::msg::Pose shoulder_origin;
  tf2::convert(init_state->getGlobalLinkTransform("l_upper_arm"), shoulder_origin);
  //arm max length, y offset, z offset from base link
  engine_.init(shoulder_origin.position.y, shoulder_origin.position.z);
  ik_.init(kinematic_model_);

  callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DynupNode::onSetParameters, this, _1));

  joint_goal_publisher_ = this->create_publisher<bitbots_msgs::msg::JointCommand>("dynup_motor_goals", 1);
  debug_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("debug_markers", 1);
  cop_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1, std::bind(&DynupNode::imuCallback, this, _1));
  joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(&DynupNode::jointStateCallback, this, _1));

  server_free_ = true;
  this->action_server_ = rclcpp_action::create_server<DynupGoal>(
    this,
    "dynup",
    std::bind(&DynupNode::goalCb, this, _1, _2),
    std::bind(&DynupNode::cancelCb, this, _1),
    std::bind(&DynupNode::acceptedCb, this, _1));
  RCLCPP_INFO(this->get_logger(),"Initialized DynUp and waiting for actions");
}

bitbots_msgs::msg::JointCommand DynupNode::step(double dt,
                                          const sensor_msgs::msg::Imu::SharedPtr imu_msg,
                                          const sensor_msgs::msg::JointState::SharedPtr jointstate_msg) {
    // method for python interface. take all messages as parameters instead of using ROS
    imuCallback(imu_msg);
    jointStateCallback(jointstate_msg);
    // update dynup engine response
    bitbots_msgs::msg::JointCommand joint_goals = step(dt);
    return joint_goals;
}

bitbots_msgs::msg::JointCommand DynupNode::step(double dt) {
    if (dt <= 0) {
        dt = 0.001;
    }
    DynupResponse response = engine_.update(dt);
    stabilizer_.setRSoleToTrunk(tf_buffer_->lookupTransform(r_sole_frame_, base_link_frame_, rclcpp::Time(0)));
    DynupResponse stabilized_response = stabilizer_.stabilize(response, rclcpp::Duration::from_nanoseconds(1e9*dt));
    bitbots_splines::JointGoals goals = ik_.calculate(stabilized_response);
    if (goals.first.empty()) {
        failed_tick_counter_++;
    }
    if (stabilizer_.isStable()) {
        stable_duration_ += 1;
    } else {
        stable_duration_ = 0;
    }
    return createGoalMsg(goals);
}

geometry_msgs::msg::PoseArray DynupNode::step_open_loop(double dt) {
    DynupNode::step(dt);
    geometry_msgs::msg::PoseArray pose_array;
    bitbots_dynup::msg::DynupPoses pose_msg = DynupNode::getCurrentPoses();
    pose_array.poses = {pose_msg.l_leg_pose, pose_msg.r_leg_pose, pose_msg.l_arm_pose, pose_msg.r_arm_pose};
    return pose_array;
}

void DynupNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr jointstates) {
  ik_.setCurrentJointStates(jointstates);
}

void DynupNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  stabilizer_.setImu(msg);
}

rcl_interfaces::msg::SetParametersResult DynupNode::onSetParameters(const std::vector<rclcpp::Parameter> &parameters) {
  auto params = this->get_parameters(param_names_);
  for (auto& param : params) {
    params_[param.get_name()] = param;
  }
  engine_rate_ = params_["engine_rate"].get_value<int>();
  debug_ = params_["display_debug"].get_value<bool>();

  engine_.setParams(params_);
  stabilizer_.setParams(params_);
  ik_.useStabilizing(params_["stabilizing"].get_value<bool>());

  VisualizationParams viz_params = VisualizationParams();
  viz_params.spline_smoothness = params_["spline_smoothness"].get_value<int>();
  visualizer_.setParams(viz_params);

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void DynupNode::reset(int time) {
    engine_.reset(time);
    ik_.reset();
    stabilizer_.reset();
}

void DynupNode::execute(const std::shared_ptr<DynupGoalHandle> goal_handle) {
  RCLCPP_INFO(this->get_logger(),"Dynup accepted new goal");
  const auto goal = goal_handle->get_goal();
  reset();
  last_ros_update_time_ = 0;
  start_time_ = this->get_clock()->now().seconds();
  bitbots_dynup::msg::DynupPoses poses = getCurrentPoses();
  if (poses.header.stamp.nanosec != 0) {
    DynupRequest request;
    request.direction = goal->direction;
    ik_.setDirection(request.direction);
    request.l_foot_pose = poses.l_leg_pose;
    request.r_foot_pose = poses.r_leg_pose;
    request.l_hand_pose = poses.l_arm_pose;
    request.r_hand_pose = poses.r_arm_pose;
    engine_.setGoals(request);
    if (debug_) {
      visualizer_.displaySplines(engine_.getRFootSplines(), base_link_frame_);
      visualizer_.displaySplines(engine_.getLFootSplines(), r_sole_frame_);
      // Workaround for an error in the Visualizer. TODO
      if (request.direction == "front" || request.direction == "back") {
        visualizer_.displaySplines(engine_.getLHandSplines(), base_link_frame_);
        visualizer_.displaySplines(engine_.getRHandSplines(), base_link_frame_);
      }
    }
    loopEngine(engine_rate_, goal_handle);
  } else {
    RCLCPP_ERROR(this->get_logger(),"Could not determine positions! Aborting standup.");
    bitbots_msgs::action::Dynup_Result::SharedPtr r = std::make_shared<bitbots_msgs::action::Dynup_Result>();
    r->successful = false;
    server_free_ = true;
    goal_handle->abort(r);
  }
}

rclcpp_action::GoalResponse DynupNode::goalCb(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const DynupGoal::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  if(server_free_) {
    server_free_ = false;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  else {
    RCLCPP_WARN(this->get_logger(), "Dynup is busy, goal rejected!");
    return rclcpp_action::GoalResponse::REJECT;
  }
}

rclcpp_action::CancelResponse DynupNode::cancelCb(
    const std::shared_ptr<DynupGoalHandle> goal) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal;
  server_free_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DynupNode::acceptedCb(const std::shared_ptr<DynupGoalHandle> goal)
{
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DynupNode::execute, this, _1), goal}.detach();
}

double DynupNode::getTimeDelta() {
  // compute actual time delta that happened
  double dt;
  double current_ros_time = this->get_clock()->now().seconds();

  // first call needs to be handled specially
  if (last_ros_update_time_ == 0) {
    last_ros_update_time_ = current_ros_time;
    return 0.001;
  }
  dt = current_ros_time - last_ros_update_time_;
  // this can happen due to floating point precision or simulation issues. will be catched later
  if (dt == 0) {
    RCLCPP_WARN_ONCE(this->get_logger(),
        "dt was 0. this can happen in simulation if your update rate is higher than the simulators. This warning is only displayed once!");
  }
  last_ros_update_time_ = current_ros_time;
  return dt;
}

void DynupNode::loopEngine(int loop_rate, std::shared_ptr<DynupGoalHandle> goal_handle) {
  auto result = std::make_shared<DynupGoal::Result>();
  double dt;
  bitbots_msgs::msg::JointCommand msg;
  /* Do the loop as long as nothing cancels it */
  while (rclcpp::ok()) {
    rclcpp::Time startTime = this->get_clock()->now();
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    dt = getTimeDelta();
    msg = step(dt);
    auto feedback = std::make_shared<bitbots_msgs::action::Dynup_Feedback>();
    feedback->percent_done = engine_.getPercentDone();
    goal_handle->publish_feedback(feedback);
    if (feedback->percent_done >= 100 && (stable_duration_ >= params_["stable_duration"].get_value<int>() || !(params_["stabilizing"].get_value<bool>()) ||
                                        (this->get_clock()->now().seconds() - start_time_ >= engine_.getDuration() + params_["stabilization_timeout"].get_value<double>()))) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Completed dynup with " << failed_tick_counter_ << " failed ticks.");
        result->successful = true;
        server_free_ = true;
        goal_handle->succeed(result);
        return;
    }
    if (msg.joint_names.empty()) {
        break;
    }
    joint_goal_publisher_->publish(msg);
    this->get_clock()->sleep_until(startTime + rclcpp::Duration::from_nanoseconds(1e9 / loop_rate));
  }
}

bitbots_dynup::msg::DynupPoses DynupNode::getCurrentPoses() {
  rclcpp::Time time = this->get_clock()->now();
  /* Transform the left foot into the right foot frame and all other splines into the base link frame*/
  bitbots_dynup::msg::DynupPoses msg;
  try {
    //0.2 second timeout for transformations
    geometry_msgs::msg::Transform l_foot_transformed = tf_buffer_->lookupTransform(r_sole_frame_, l_sole_frame_, time, tf2::durationFromSec(0.2)).transform;
    geometry_msgs::msg::Transform r_foot_transformed = tf_buffer_->lookupTransform(base_link_frame_, r_sole_frame_, time, tf2::durationFromSec(0.2)).transform;
    geometry_msgs::msg::Transform l_hand_transformed = tf_buffer_->lookupTransform(base_link_frame_, l_wrist_frame_, time, tf2::durationFromSec(0.2)).transform;
    geometry_msgs::msg::Transform r_hand_transformed = tf_buffer_->lookupTransform(base_link_frame_, r_wrist_frame_, time, tf2::durationFromSec(0.2)).transform;

    std::function transform2pose =  [](geometry_msgs::msg::Transform transform) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = transform.translation.x;
      pose.position.y = transform.translation.y;
      pose.position.z = transform.translation.z;
      pose.orientation = transform.rotation;
      return pose;
    };

    msg.l_leg_pose = transform2pose(l_foot_transformed);
    msg.r_leg_pose = transform2pose(r_foot_transformed);
    msg.l_arm_pose = transform2pose(l_hand_transformed);
    msg.r_arm_pose = transform2pose(r_hand_transformed);
    msg.header.stamp = this->get_clock()->now();
    return msg;
  } catch (tf2::TransformException &exc) {
    RCLCPP_ERROR_STREAM(this->get_logger(), exc.what());
    return msg;
  }

}

bitbots_msgs::msg::JointCommand DynupNode::createGoalMsg(const bitbots_splines::JointGoals &goals) {
  /* Construct JointCommand message */
  bitbots_msgs::msg::JointCommand command;
  command.header.stamp = this->get_clock()->now();

  /*
   * Since our JointGoals type is a vector of strings
   *  combined with a vector of numbers (motor name -> target position)
   *  and bitbots_msgs::msg::JointCommand needs both vectors as well,
   *  we can just assign them
   */
  command.joint_names = goals.first;
  command.positions = goals.second;

  /* And because we are setting position goals and not movement goals, these vectors are set to -1.0*/
  std::vector<double> vels(goals.first.size(), -1.0);
  std::vector<double> accs(goals.first.size(), -1.0);
  std::vector<double> pwms(goals.first.size(), -1.0);
  command.velocities = vels;
  command.accelerations = accs;
  command.max_currents = pwms;

  return command;
}

DynupEngine *DynupNode::getEngine() {
    return &engine_;
}

DynupIK *DynupNode::getIK() {
    return &ik_;
}

}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // init node
  auto node = std::make_shared<bitbots_dynup::DynupNode>();
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  exec.spin();
  rclcpp::shutdown();
}
