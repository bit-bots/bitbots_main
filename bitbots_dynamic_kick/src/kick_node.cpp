#include "bitbots_dynamic_kick/kick_node.h"

namespace bitbots_dynamic_kick {

KickNode::KickNode(const std::string &ns) :
    private_node_handle_("~"),
    server_(node_handle_, "dynamic_kick", boost::bind(&KickNode::executeCb, this, _1), false),
    visualizer_(ns + "debug/dynamic_kick"),
    listener_(tf_buffer_),
    robot_model_loader_(ns + "robot_description", false) {
  private_node_handle_.param<std::string>("base_link_frame", base_link_frame_, "base_link");
  private_node_handle_.param<std::string>("base_footprint_frame", base_footprint_frame_, "base_footprint");
  private_node_handle_.param<std::string>("r_sole_frame", r_sole_frame_, "r_sole");
  private_node_handle_.param<std::string>("l_sole_frame", l_sole_frame_, "l_sole");

  unstable_config_ = getUnstableConfig();

  /* load MoveIt! model */
  robot_model_loader_.loadKinematicsSolvers(std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>());
  robot_model::RobotModelPtr kinematic_model = robot_model_loader_.getModel();
  if (!kinematic_model) {
    ROS_FATAL("No robot model loaded, killing dynamic kick.");
    exit(1);
  }
  stabilizer_.setRobotModel(kinematic_model);
  ik_.init(kinematic_model);

  /* this debug variable represents the goal state of the robot, i.e. what the ik goals are */
  goal_state_.reset(new robot_state::RobotState(kinematic_model));
  /* this variable represents the current state of the robot, retrieved from joint states */
  current_state_.reset(new robot_state::RobotState(kinematic_model));
  engine_.setRobotState(current_state_);

  joint_goal_publisher_ = node_handle_.advertise<bitbots_msgs::JointCommand>("kick_motor_goals", 1);
  support_foot_publisher_ =
      node_handle_.advertise<bitbots_msgs::SupportState>("dynamic_kick_support_state", 1, /* latch = */ true);
  cop_l_subscriber_ = node_handle_.subscribe("cop_l", 1, &KickNode::copLCallback, this);
  cop_r_subscriber_ = node_handle_.subscribe("cop_r", 1, &KickNode::copRCallback, this);
  joint_state_subscriber_ = node_handle_.subscribe("joint_states", 1, &KickNode::jointStateCallback, this);
  server_.start();
}

void KickNode::copLCallback(const geometry_msgs::PointStamped &cop) {
  if (cop.header.frame_id != l_sole_frame_) {
    ROS_ERROR_STREAM("cop_l not in " << l_sole_frame_ << " frame! Stabilizing will not work.");
  }
  stabilizer_.cop_left = cop.point;
}

void KickNode::copRCallback(const geometry_msgs::PointStamped &cop) {
  if (cop.header.frame_id != r_sole_frame_) {
    ROS_ERROR_STREAM("cop_r not in " << r_sole_frame_ << " frame! Stabilizing will not work.");
  }
  stabilizer_.cop_right = cop.point;
}

void KickNode::jointStateCallback(const sensor_msgs::JointState &joint_states) {
  for (size_t i = 0; i < joint_states.name.size(); ++i) {
    current_state_->setJointPositions(joint_states.name[i], &joint_states.position[i]);
  }
}

DynamicKickConfig KickNode::getUnstableConfig() {
  DynamicKickConfig config;
  ros::NodeHandle unstable_node_handle("~/unstable");
  private_node_handle_.param<int>("engine_rate", config.engine_rate, 0);

  unstable_node_handle.param<double>("foot_rise", config.foot_rise, 0);
  unstable_node_handle.param<double>("foot_distance", config.foot_distance, 0);
  unstable_node_handle.param<double>("kick_windup_distance", config.kick_windup_distance, 0);
  unstable_node_handle.param<double>("trunk_height", config.trunk_height, 0);
  unstable_node_handle.param<double>("trunk_roll", config.trunk_roll, 0);
  unstable_node_handle.param<double>("trunk_pitch", config.trunk_pitch, 0);
  unstable_node_handle.param<double>("trunk_yaw", config.trunk_yaw, 0);

  unstable_node_handle.param<double>("move_trunk_time", config.move_trunk_time, 0);
  unstable_node_handle.param<double>("raise_foot_time", config.raise_foot_time, 0);
  unstable_node_handle.param<double>("move_to_ball_time", config.move_to_ball_time, 0);
  unstable_node_handle.param<double>("kick_time", config.kick_time, 0);
  unstable_node_handle.param<double>("move_back_time", config.move_back_time, 0);
  unstable_node_handle.param<double>("lower_foot_time", config.lower_foot_time, 0);
  unstable_node_handle.param<double>("move_trunk_back_time", config.move_trunk_back_time, 0);

  private_node_handle_.param<double>("choose_foot_corridor_width", config.choose_foot_corridor_width, 0);

  unstable_node_handle.param<bool>("use_center_of_pressure", config.use_center_of_pressure, false);
  unstable_node_handle.param<double>("stabilizing_point_x", config.stabilizing_point_x, 0);
  unstable_node_handle.param<double>("stabilizing_point_y", config.stabilizing_point_y, 0);

  private_node_handle_.param<int>("spline_smoothness", config.spline_smoothness, 0);

  return config;
}

void KickNode::reconfigureCallback(bitbots_dynamic_kick::DynamicKickConfig &config, uint32_t level) {
  if (level != 9999) {
    // The level is set to 9999 for the unstable kick, so we don't update the normal config in that case
    normal_config_ = config;
  }
  engine_rate_ = config.engine_rate;

  KickParams params = KickParams();
  params.foot_rise = config.foot_rise;
  params.foot_distance = config.foot_distance;
  params.kick_windup_distance = config.kick_windup_distance;
  params.trunk_height = config.trunk_height;
  params.trunk_roll = config.trunk_roll;
  params.trunk_pitch = config.trunk_pitch;
  params.trunk_yaw = config.trunk_yaw;
  params.move_trunk_time = config.move_trunk_time;
  params.raise_foot_time = config.raise_foot_time;
  params.move_to_ball_time = config.move_to_ball_time;
  params.kick_time = config.kick_time;
  params.move_back_time = config.move_back_time;
  params.lower_foot_time = config.lower_foot_time;
  params.move_trunk_back_time = config.move_trunk_back_time;
  params.stabilizing_point_x = config.stabilizing_point_x;
  params.stabilizing_point_y = config.stabilizing_point_y;
  params.choose_foot_corridor_width = config.choose_foot_corridor_width;
  engine_.setParams(params);

  stabilizer_.useCop(config.use_center_of_pressure);

  VisualizationParams viz_params = VisualizationParams();
  viz_params.spline_smoothness = config.spline_smoothness;
  visualizer_.setParams(viz_params);
}

bool KickNode::init(const bitbots_msgs::KickGoal &goal_msg,
                    std::string &error_string) {
  /* currently, the ball must always be in the base_footprint frame */
  if (goal_msg.header.frame_id != base_footprint_frame_) {
    ROS_ERROR_STREAM("Goal should be in " << base_footprint_frame_ << " frame");
    error_string = std::string("Goal should be in ") + base_footprint_frame_ + std::string(" frame");
    return false;
  }

  if (goal_msg.unstable) {
    reconfigureCallback(unstable_config_, 9999);
  } else {
    reconfigureCallback(normal_config_.value(), 0);
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
  tf2::convert(goal_msg.ball_position, goals.ball_position);
  tf2::convert(goal_msg.kick_direction, goals.kick_direction);
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

void KickNode::executeCb(const bitbots_msgs::KickGoalConstPtr &goal) {
  // TODO: maybe switch to goal callback to be able to reject goals properly
  ROS_INFO("Accepted new goal");

  /* get transform to base_footprint */
  geometry_msgs::TransformStamped goal_frame_to_base_footprint =
      tf_buffer_.lookupTransform(base_footprint_frame_, goal->header.frame_id, ros::Time(0));
  geometry_msgs::Point base_footprint_ball_position;
  tf2::doTransform(goal->ball_position, base_footprint_ball_position, goal_frame_to_base_footprint);
  bitbots_msgs::KickGoal base_footprint_kick_goal = *goal;
  base_footprint_kick_goal.ball_position = base_footprint_ball_position;
  base_footprint_kick_goal.header.frame_id = base_footprint_frame_;

  /* pass everything to the init function */
  std::string error_string;
  bool success = init(base_footprint_kick_goal, error_string);
  /* there was an error, abort the kick */
  if (!success) {
    bitbots_msgs::KickResult result;
    result.result = bitbots_msgs::KickResult::REJECTED;
    server_.setAborted(result, error_string);
  }

  /* everything is set up, start calculating now until the kick is finished or an error occurs */
  ros::Rate loop_rate(engine_rate_);
  loopEngine(loop_rate);

  /* Figure out the reason why loopEngine() returned and act accordingly */
  if (server_.isPreemptRequested()) {
    /* Confirm that we canceled the previous goal */
    ROS_INFO("Cancelled old goal");
    bitbots_msgs::KickResult result;
    result.result = bitbots_msgs::KickResult::ABORTED;
    server_.setPreempted(result);
  } else {
    /* Publish results */
    ROS_INFO("Done kicking ball");
    bitbots_msgs::KickResult result;
    result.result = bitbots_msgs::KickResult::SUCCESS;
    server_.setSucceeded(result);
  }
}

double KickNode::getTimeDelta() {
  // compute actual time delta that happened
  double dt;
  double current_ros_time = ros::Time::now().toSec();

  // first call needs to be handled specially
  if (last_ros_update_time_ == 0) {
    last_ros_update_time_ = current_ros_time;
    return 0.001;
  }

  dt = current_ros_time - last_ros_update_time_;
  // this can happen due to floating point precision or in simulation
  if (dt == 0) {
    ROS_WARN("dynamic kick: dt was 0");
    dt = 0.001;
  }
  last_ros_update_time_ = current_ros_time;

  return dt;
}

void KickNode::loopEngine(ros::Rate loop_rate) {
  /* Do the loop as long as nothing cancels it */
  double dt;
  while (server_.isActive() && !server_.isPreemptRequested()) {
    ros::spinOnce();
    if (loop_rate.sleep()) {
      dt = getTimeDelta();
      std::optional<bitbots_splines::JointGoals> motor_goals = kickStep(dt);

      /* Publish feedback to the client */
      bitbots_msgs::KickFeedback feedback;
      feedback.percent_done = engine_.getPercentDone();
      feedback.chosen_foot = engine_.isLeftKick() ?
                             bitbots_msgs::KickFeedback::FOOT_LEFT : bitbots_msgs::KickFeedback::FOOT_RIGHT;
      server_.publishFeedback(feedback);

      if (feedback.percent_done >= 100) {
        break;
      }
      joint_goal_publisher_.publish(getJointCommand(motor_goals.value()));
    } else {
      usleep(1);
    }
  }
}

bitbots_splines::JointGoals KickNode::kickStep(double dt) {
  KickPositions positions = engine_.update(dt);
  // TODO: should positions be an std::optional? how are errors represented?
  KickPositions stabilized_positions = stabilizer_.stabilize(positions, ros::Duration(dt));
  bitbots_splines::JointGoals motor_goals = ik_.calculate(stabilized_positions);

  /* visualization of the values calculated above */
  for (size_t i = 0; i < motor_goals.first.size(); ++i) {
    goal_state_->setJointPositions(motor_goals.first[i], &motor_goals.second[i]);
  }
  visualizer_.publishGoals(positions, stabilized_positions, goal_state_, engine_.getPhase());
  publishSupportFoot(engine_.isLeftKick());

  return motor_goals;
}

bitbots_msgs::JointCommand KickNode::getJointCommand(const bitbots_splines::JointGoals &goals) {
  /* Construct JointCommand message */
  bitbots_msgs::JointCommand command;
  command.header.stamp = ros::Time::now();

  /*
   * Since our JointGoals type is a vector of strings
   *  combined with a vector of numbers (motor name -> target position)
   *  and bitbots_msgs::JointCommand needs both vectors as well,
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
  bitbots_msgs::SupportState msg;
  msg.state = is_left_kick ? bitbots_msgs::SupportState::RIGHT : bitbots_msgs::SupportState::LEFT;
  // only publish one time per kick
  if (!was_support_foot_published_) {
    support_foot_publisher_.publish(msg);
    was_support_foot_published_ = true;
  }
}

bitbots_msgs::JointCommand KickNode::stepWrapper(double dt) {
  /* with stabilizing, we can call some callbacks here */
  bitbots_splines::JointGoals goals = kickStep(dt);
  if (engine_.getPercentDone() < 100) {
    return getJointCommand(goals);
  } else {
    return {};
  }
}

double KickNode::getProgress() {
  return engine_.getPercentDone() / 100.0;
}

geometry_msgs::Pose KickNode::getTrunkPose() {
  return engine_.getTrunkPose();
}

bool KickNode::isLeftKick() {
  return engine_.isLeftKick();
}

}

int main(int argc, char *argv[]) {
  /* Setup ROS node */
  ros::init(argc, argv, "dynamic_kick");
  bitbots_dynamic_kick::KickNode node;

  /* Setup dynamic_reconfigure */
  dynamic_reconfigure::Server<bitbots_dynamic_kick::DynamicKickConfig> dyn_reconf_server;
  dyn_reconf_server.setCallback(boost::bind(&bitbots_dynamic_kick::KickNode::reconfigureCallback, &node, _1, _2));

  ros::spin();
}
