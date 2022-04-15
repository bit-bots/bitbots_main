#include "bitbots_dynamic_kick/kick_node.h"

namespace bitbots_dynamic_kick {
using namespace std::chrono_literals;

KickNode::KickNode(const std::string &ns) :
    node_(std::make_shared<rclcpp::Node>(ns + "dynamic_kick")),
    stabilizer_(ns),
    visualizer_(ns + "debug/dynamic_kick", node_) {


  // get all kinematics parameters from the move_group node if they are not set manually via constructor
  std::string check_kinematic_parameters;
  if (!node_->get_parameter("robot_description_kinematics.LeftLeg.kinematics_solver", check_kinematic_parameters)) {
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "/move_group");
    while (!parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO_THROTTLE(node_->get_logger(),
                           *node_->get_clock(),
                           10 * 1e9,
                           "Can't copy parameters from move_group node. Service not available, waiting again...");
    }
    rcl_interfaces::msg::ListParametersResult
        parameter_list = parameters_client->list_parameters({"robot_description_kinematics"}, 10);
    auto copied_parameters = parameters_client->get_parameters(parameter_list.names);
    // set the parameters to our node
    node_->set_parameters(copied_parameters);
  }

  node_->declare_parameter<std::string>("base_link_frame", "base_link");
  node_->get_parameter("base_link_frame", base_link_frame_);
  node_->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
  node_->get_parameter("base_footprint_frame", base_footprint_frame_);
  node_->declare_parameter<std::string>("r_sole_frame", "r_sole");
  node_->get_parameter("r_sole_frame", r_sole_frame_);
  node_->declare_parameter<std::string>("l_sole_frame", "l_sole");
  node_->get_parameter("l_sole_frame", l_sole_frame_);

  VisualizationParams viz_params = VisualizationParams();
  node_->declare_parameter<int>("spline_smoothness", 0);
  node_->get_parameter("spline_smoothness", viz_params.spline_smoothness);
  visualizer_.setParams(viz_params);
  node_->declare_parameter<int>("engine_rate", 0);
  node_->get_parameter("engine_rate", engine_rate_);
  bool use_center_of_pressure;
  node_->declare_parameter<bool>("use_center_of_pressure", false);
  node_->get_parameter("use_center_of_pressure", use_center_of_pressure);
  stabilizer_.useCop(use_center_of_pressure);

  normal_config_ = KickParams();
  node_->declare_parameter<double>("foot_rise", 0);
  node_->get_parameter("foot_rise", normal_config_.foot_rise);
  node_->declare_parameter<double>("foot_distance", 0);
  node_->get_parameter("foot_distance", normal_config_.foot_distance);
  node_->declare_parameter<double>("kick_windup_distance", 0);
  node_->get_parameter("kick_windup_distance", normal_config_.kick_windup_distance);
  node_->declare_parameter<double>("trunk_height", 0);
  node_->get_parameter("trunk_height", normal_config_.trunk_height);
  node_->declare_parameter<double>("trunk_roll", 0);
  node_->get_parameter("trunk_roll", normal_config_.trunk_roll);
  node_->declare_parameter<double>("trunk_pitch", 0);
  node_->get_parameter("trunk_pitch", normal_config_.trunk_pitch);
  node_->declare_parameter<double>("trunk_yaw", 0);
  node_->get_parameter("trunk_yaw", normal_config_.trunk_yaw);
  node_->declare_parameter<double>("move_trunk_time", 0);
  node_->get_parameter("move_trunk_time", normal_config_.move_trunk_time);
  node_->declare_parameter<double>("raise_foot_time", 0);
  node_->get_parameter("raise_foot_time", normal_config_.raise_foot_time);
  node_->declare_parameter<double>("move_to_ball_time", 0);
  node_->get_parameter("move_to_ball_time", normal_config_.move_to_ball_time);
  node_->declare_parameter<double>("kick_time", 0);
  node_->get_parameter("kick_time", normal_config_.kick_time);
  node_->declare_parameter<double>("move_back_time", 0);
  node_->get_parameter("move_back_time", normal_config_.move_back_time);
  node_->declare_parameter<double>("lower_foot_time", 0);
  node_->get_parameter("lower_foot_time", normal_config_.lower_foot_time);
  node_->declare_parameter<double>("move_trunk_back_time", 0);
  node_->get_parameter("move_trunk_back_time", normal_config_.move_trunk_back_time);
  node_->declare_parameter<double>("choose_foot_corridor_width", 0);
  node_->get_parameter("choose_foot_corridor_width", normal_config_.choose_foot_corridor_width);
  node_->declare_parameter<double>("stabilizing_point_x", 0);
  node_->get_parameter("stabilizing_point_x", normal_config_.stabilizing_point_x);
  node_->declare_parameter<double>("stabilizing_point_y", 0);
  node_->get_parameter("stabilizing_point_y", normal_config_.stabilizing_point_y);

  unstable_config_ = KickParams();
  node_->declare_parameter<double>("unstable.foot_rise", 0);
  node_->get_parameter("unstable.foot_rise", unstable_config_.foot_rise);
  node_->declare_parameter<double>("unstable.foot_distance", 0);
  node_->get_parameter("unstable.foot_distance", unstable_config_.foot_distance);
  node_->declare_parameter<double>("unstable.kick_windup_distance", 0);
  node_->get_parameter("unstable.kick_windup_distance", unstable_config_.kick_windup_distance);
  node_->declare_parameter<double>("unstable.trunk_height", 0);
  node_->get_parameter("unstable.trunk_height", unstable_config_.trunk_height);
  node_->declare_parameter<double>("unstable.trunk_roll", 0);
  node_->get_parameter("unstable.trunk_roll", unstable_config_.trunk_roll);
  node_->declare_parameter<double>("unstable.trunk_pitch", 0);
  node_->get_parameter("unstable.trunk_pitch", unstable_config_.trunk_pitch);
  node_->declare_parameter<double>("unstable.trunk_yaw", 0);
  node_->get_parameter("unstable.trunk_yaw", unstable_config_.trunk_yaw);
  node_->declare_parameter<double>("unstable.move_trunk_time", 0);
  node_->get_parameter("unstable.move_trunk_time", unstable_config_.move_trunk_time);
  node_->declare_parameter<double>("unstable.raise_foot_time", 0);
  node_->get_parameter("unstable.raise_foot_time", unstable_config_.raise_foot_time);
  node_->declare_parameter<double>("unstable.move_to_ball_time", 0);
  node_->get_parameter("unstable.move_to_ball_time", unstable_config_.move_to_ball_time);
  node_->declare_parameter<double>("unstable.kick_time", 0);
  node_->get_parameter("unstable.kick_time", unstable_config_.kick_time);
  node_->declare_parameter<double>("unstable.move_back_time", 0);
  node_->get_parameter("unstable.move_back_time", unstable_config_.move_back_time);
  node_->declare_parameter<double>("unstable.lower_foot_time", 0);
  node_->get_parameter("unstable.lower_foot_time", unstable_config_.lower_foot_time);
  node_->declare_parameter<double>("unstable.move_trunk_back_time", 0);
  node_->get_parameter("unstable.move_trunk_back_time", unstable_config_.move_trunk_back_time);
  node_->declare_parameter<double>("unstable.choose_foot_corridor_width", 0);
  node_->get_parameter("unstable.choose_foot_corridor_width", unstable_config_.choose_foot_corridor_width);
  node_->declare_parameter<double>("unstable.stabilizing_point_x", 0);
  node_->get_parameter("unstable.stabilizing_point_x", unstable_config_.stabilizing_point_x);
  node_->declare_parameter<double>("unstable.stabilizing_point_y", 0);
  node_->get_parameter("unstable.stabilizing_point_y", unstable_config_.stabilizing_point_y);


  /* load MoveIt! model */
  robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description", true);
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader_->getModel();
  if (!kinematic_model) {
    RCLCPP_FATAL(node_->get_logger(), "No robot model loaded, killing dynamic kick.");
    exit(1);
  }
  stabilizer_.setRobotModel(kinematic_model);
  ik_.init(kinematic_model);

  /* this debug variable represents the goal state of the robot, i.e. what the ik goals are */
  goal_state_.reset(new moveit::core::RobotState(kinematic_model));
  /* this variable represents the current state of the robot, retrieved from joint states */
  current_state_.reset(new moveit::core::RobotState(kinematic_model));
  engine_.setRobotState(current_state_);

  joint_goal_publisher_ = node_->create_publisher<bitbots_msgs::msg::JointCommand>("kick_motor_goals", 1);
  support_foot_publisher_ =
      node_->create_publisher<biped_interfaces::msg::Phase>("dynamic_kick_support_state", 1);
  cop_l_subscriber_ = node_
      ->create_subscription<geometry_msgs::msg::PointStamped>("cop_l", 1, std::bind(&KickNode::copLCallback, this, _1));
  cop_r_subscriber_ = node_
      ->create_subscription<geometry_msgs::msg::PointStamped>("cop_r", 1, std::bind(&KickNode::copRCallback, this, _1));
  joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>("joint_states",
                                                                                     1,
                                                                                     std::bind(&KickNode::jointStateCallback,
                                                                                               this,
                                                                                               _1));

  callback_handle_ = node_->add_on_set_parameters_callback(std::bind(&KickNode::onSetParameters, this, _1));

  server_ = rclcpp_action::create_server<bitbots_msgs::action::Kick>(node_,
                                                                     "dynamic_kick",
                                                                     std::bind(&KickNode::goalCb,
                                                                               this,
                                                                               std::placeholders::_1,
                                                                               std::placeholders::_2),
                                                                     std::bind(&KickNode::cancelCb,
                                                                               this,
                                                                               std::placeholders::_1),
                                                                     std::bind(&KickNode::acceptedCb,
                                                                               this,
                                                                               std::placeholders::_1));
}

void KickNode::copLCallback(const geometry_msgs::msg::PointStamped::SharedPtr cop) {
  if (cop->header.frame_id != l_sole_frame_) {
    ROS_ERROR_STREAM("cop_l not in " << l_sole_frame_ << " frame! Stabilizing will not work.");
  }
  stabilizer_.cop_left = cop->point;
}

void KickNode::copRCallback(const geometry_msgs::msg::PointStamped::SharedPtr  cop) {
  if (cop->header.frame_id != r_sole_frame_) {
    ROS_ERROR_STREAM("cop_r not in " << r_sole_frame_ << " frame! Stabilizing will not work.");
  }
  stabilizer_.cop_right = cop->point;
}

void KickNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr joint_states) {
  for (size_t i = 0; i < joint_states->name.size(); ++i) {
    current_state_->setJointPositions(joint_states->name[i], &joint_states->position[i]);
  }
}

rcl_interfaces::msg::SetParametersResult KickNode::onSetParameters(const std::vector<rclcpp::Parameter> &parameters) {
  for (const auto &parameter: parameters) {
    if (parameter.get_name() == "engine_rate") {
      engine_rate_ = parameter.as_int();
    } else if (parameter.get_name() == "foot_rise") {
      normal_config_.foot_rise = parameter.as_double();
    } else if (parameter.get_name() == "foot_distance") {
      normal_config_.foot_distance = parameter.as_double();
    } else if (parameter.get_name() == "kick_windup_distance") {
      normal_config_.kick_windup_distance = parameter.as_double();
    } else if (parameter.get_name() == "trunk_height") {
      normal_config_.trunk_height = parameter.as_double();
    } else if (parameter.get_name() == "trunk_roll") {
      normal_config_.trunk_roll = parameter.as_double();
    } else if (parameter.get_name() == "trunk_pitch") {
      normal_config_.trunk_pitch = parameter.as_double();
    } else if (parameter.get_name() == "trunk_yaw") {
      normal_config_.trunk_yaw = parameter.as_double();
    } else if (parameter.get_name() == "move_trunk_time") {
      normal_config_.move_trunk_time = parameter.as_double();
    } else if (parameter.get_name() == "raise_foot_time") {
      normal_config_.raise_foot_time = parameter.as_double();
    } else if (parameter.get_name() == "kick_windup_distance") {
      normal_config_.kick_windup_distance = parameter.as_double();
    } else if (parameter.get_name() == "move_to_ball_time") {
      normal_config_.move_to_ball_time = parameter.as_double();
    } else if (parameter.get_name() == "kick_time") {
      normal_config_.kick_time = parameter.as_double();
    } else if (parameter.get_name() == "move_back_time") {
      normal_config_.move_back_time = parameter.as_double();
    } else if (parameter.get_name() == "move_trunk_back_time") {
      normal_config_.move_trunk_back_time = parameter.as_double();
    } else if (parameter.get_name() == "stabilizing_point_x") {
      normal_config_.stabilizing_point_x = parameter.as_double();
    } else if (parameter.get_name() == "stabilizing_point_y") {
      normal_config_.stabilizing_point_y = parameter.as_double();
    } else if (parameter.get_name() == "choose_foot_corridor_width") {
      normal_config_.choose_foot_corridor_width = parameter.as_double();
    } else if (parameter.get_name() == "use_center_of_pressure") {
      stabilizer_.useCop(parameter.as_bool());
    } else if (parameter.get_name() == "spline_smoothness") {
      VisualizationParams viz_params = VisualizationParams();
      viz_params.spline_smoothness = parameter.as_int();
      visualizer_.setParams(viz_params);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Unknown parameter: %s", parameter.get_name().c_str());
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

bool KickNode::init(const bitbots_msgs::action::Kick::Goal &goal_msg,
                    std::string &error_string) {
  /* currently, the ball must always be in the base_footprint frame */
  if (goal_msg.header.frame_id != base_footprint_frame_) {
    ROS_ERROR_STREAM("Goal should be in " << base_footprint_frame_ << " frame");
    error_string = std::string("Goal should be in ") + base_footprint_frame_ + std::string(" frame");
    return false;
  }

  if (goal_msg.unstable) {
    engine_.setParams(unstable_config_);
  } else {
    engine_.setParams(normal_config_);
  }

  /* reset the current state */
  last_ros_update_time_ = 0;
  was_support_foot_published_ = false;
  engine_.reset();
  stabilizer_.reset();
  ik_.reset();

  /* get trunk to base footprint transform, assume left and right foot are next to each other (same x and z) */
  Eigen::Isometry3d trunk_to_l_sole = current_state_->getGlobalLinkTransform("l_sole");
  Eigen::Isometry3d trunk_to_r_sole = current_state_->getGlobalLinkTransform("r_sole");
  Eigen::Isometry3d trunk_to_base_footprint = trunk_to_l_sole;
  trunk_to_base_footprint.translation().y() =
      (trunk_to_r_sole.translation().y() + trunk_to_l_sole.translation().y()) / 2.0;

  /* Set engines goal_msg and start calculating */
  KickGoals goals;
  goals.ball_position = {goal_msg.ball_position.x, goal_msg.ball_position.y, goal_msg.ball_position.z};
  goals.kick_direction = {goal_msg.kick_direction.x, goal_msg.kick_direction.y, goal_msg.kick_direction.z, goal_msg.kick_direction.w};
  goals.kick_speed = goal_msg.kick_speed;
  goals.trunk_to_base_footprint = trunk_to_base_footprint;
  engine_.setGoals(goals);

  /* visualization */
  visualizer_.displayReceivedGoal(goal_msg);
  visualizer_.displayWindupPoint(engine_.getWindupPoint(), (engine_.isLeftKick()) ? r_sole_frame_ : l_sole_frame_);
  visualizer_.displayFlyingSplines(engine_.getFlyingSplines(), (engine_.isLeftKick()) ? r_sole_frame_ : l_sole_frame_);
  visualizer_.displayTrunkSplines(engine_.getTrunkSplines(), (engine_.isLeftKick() ? r_sole_frame_ : l_sole_frame_));

  return true;
}

rclcpp_action::CancelResponse KickNode::cancelCb(std::shared_ptr<rclcpp_action::ServerGoalHandle<bitbots_msgs::action::Kick>> goal) {
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
  (void) goal;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void KickNode::acceptedCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bitbots_msgs::action::Kick>> goal) {
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&KickNode::executeCb, this, _1), goal}.detach();
}

rclcpp_action::GoalResponse KickNode::goalCb(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const bitbots_msgs::action::Kick::Goal> goal) {
  RCLCPP_INFO(node_->get_logger(), "Received goal request");
  (void) uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void KickNode::executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bitbots_msgs::action::Kick>> goal_handle) {
  // TODO: maybe switch to goal callback to be able to reject goals properly
  RCLCPP_INFO(node_->get_logger(), "Accepted new goal");
  const auto goal = goal_handle->get_goal();
  /* get transform to base_footprint */
  geometry_msgs::msg::TransformStamped goal_frame_to_base_footprint =
      tf_buffer_->lookupTransform(base_footprint_frame_, goal->header.frame_id, rclcpp::Time(0));
  geometry_msgs::msg::Point base_footprint_ball_position;
  tf2::doTransform(goal->ball_position, base_footprint_ball_position, goal_frame_to_base_footprint);
  bitbots_msgs::action::Kick::Goal base_footprint_kick_goal = *goal;
  base_footprint_kick_goal.ball_position = base_footprint_ball_position;
  base_footprint_kick_goal.header.frame_id = base_footprint_frame_;

  /* pass everything to the init function */
  std::string error_string;
  bool success = init(base_footprint_kick_goal, error_string);
  /* there was an error, abort the kick */
  if (!success) {
    bitbots_msgs::action::Kick::Result::SharedPtr result;
    result->result = bitbots_msgs::action::Kick::Result::REJECTED;
    goal_handle->canceled(result);
  }

  /* everything is set up, start calculating now until the kick is finished or an error occurs */
  loopEngine(goal_handle);

  /* Figure out the reason why loopEngine() returned and act accordingly */
  if (goal_handle->is_canceling()) {
    /* Confirm that we canceled the previous goal */
    RCLCPP_INFO(node_->get_logger(), "Cancelled old goal");
    bitbots_msgs::action::Kick::Result::SharedPtr result;
    result->result = bitbots_msgs::action::Kick::Result::ABORTED;
    goal_handle->abort(result);
  } else {
    /* Publish results */
    RCLCPP_INFO(node_->get_logger(), "Done kicking ball");
    bitbots_msgs::action::Kick::Result::SharedPtr result;
    result->result = bitbots_msgs::action::Kick::Result::SUCCESS;
    goal_handle->succeed(result);
  }
}

double KickNode::getTimeDelta() {
  // compute actual time delta that happened
  double dt;
  double current_ros_time = node_->get_clock()->now().seconds();

  // first call needs to be handled specially
  if (last_ros_update_time_ == 0) {
    last_ros_update_time_ = current_ros_time;
    return 0.001;
  }

  dt = current_ros_time - last_ros_update_time_;
  // this can happen due to floating point precision or in simulation
  if (dt == 0) {
    RCLCPP_WARN(node_->get_logger(), "dynamic kick: dt was 0");
    dt = 0.001;
  }
  last_ros_update_time_ = current_ros_time;

  return dt;
}

void KickNode::loopEngine(const std::shared_ptr<rclcpp_action::ServerGoalHandle<bitbots_msgs::action::Kick>> goal_handle) {
  /* Do the loop as long as nothing cancels it */
  double dt;
  rclcpp::Time next_loop_time;
  rclcpp::Time last_time = node_->get_clock()->now();
  while (rclcpp::ok()) {
    rclcpp::spin_some(node_->get_node_base_interface());
    next_loop_time = last_time + rclcpp::Duration::from_seconds(1.0 / engine_rate_);
    last_time = node_->get_clock()->now();
    if (node_->get_clock()->sleep_until(next_loop_time)) {
      dt = getTimeDelta();
      std::optional<bitbots_splines::JointGoals> motor_goals = kickStep(dt);

      /* Publish feedback to the client */
      auto feedback = std::make_shared<bitbots_msgs::action::Kick::Feedback>();
      feedback->percent_done = engine_.getPercentDone();
      feedback->chosen_foot = engine_.isLeftKick() ?
                              bitbots_msgs::action::Kick::Feedback::FOOT_LEFT
                                                   : bitbots_msgs::action::Kick::Feedback::FOOT_RIGHT;
      goal_handle->publish_feedback(feedback);

      if (feedback->percent_done >= 100) {
        break;
      }
      joint_goal_publisher_->publish(getJointCommand(motor_goals.value()));
    } else {
      usleep(1);
    }
  }
}

bitbots_splines::JointGoals KickNode::kickStep(double dt) {
  KickPositions positions = engine_.update(dt);
  // TODO: should positions be an std::optional? how are errors represented?
  KickPositions stabilized_positions = stabilizer_.stabilize(positions, rclcpp::Duration::from_nanoseconds(1e9 * dt));
  bitbots_splines::JointGoals motor_goals = ik_.calculate(stabilized_positions);

  /* visualization of the values calculated above */
  for (size_t i = 0; i < motor_goals.first.size(); ++i) {
    goal_state_->setJointPositions(motor_goals.first[i], &motor_goals.second[i]);
  }
  visualizer_.publishGoals(positions, stabilized_positions, goal_state_, engine_.getPhase());
  publishSupportFoot(engine_.isLeftKick());

  return motor_goals;
}

bitbots_msgs::msg::JointCommand KickNode::getJointCommand(const bitbots_splines::JointGoals &goals) {
  /* Construct JointCommand message */
  bitbots_msgs::msg::JointCommand command;
  command.header.stamp = node_->get_clock()->now();

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

void KickNode::publishSupportFoot(bool is_left_kick) {
  biped_interfaces::msg::Phase msg;
  msg.phase = is_left_kick ? biped_interfaces::msg::Phase::RIGHT_STANCE : biped_interfaces::msg::Phase::LEFT_STANCE;
  // only publish one time per kick
  if (!was_support_foot_published_) {
    support_foot_publisher_->publish(msg);
    was_support_foot_published_ = true;
  }
}

bitbots_msgs::msg::JointCommand KickNode::stepWrapper(double dt) {
  /* with stabilizing, we can call some callbacks here */
  bitbots_splines::JointGoals goals = kickStep(dt);
  if (engine_.getPercentDone() < 100) {
    return getJointCommand(goals);
  } else {
    return bitbots_msgs::msg::JointCommand();
  }
}

double KickNode::getProgress() {
  return engine_.getPercentDone() / 100.0;
}

geometry_msgs::msg::Pose KickNode::getTrunkPose() {
  return engine_.getTrunkPose();
}

bool KickNode::isLeftKick() {
  return engine_.isLeftKick();
}

}

int main(int argc, char *argv[]) {
  /* Setup ROS node */
  rclcpp::init(argc, argv);
  bitbots_dynamic_kick::KickNode node;

  ros::spin();
}
