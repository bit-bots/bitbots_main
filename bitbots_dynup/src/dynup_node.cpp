#include "bitbots_dynup/dynup_node.h"


namespace bitbots_dynup {

DynUpNode::DynUpNode() :
    server_(node_handle_, "dynup", boost::bind(&DynUpNode::executeCb, this, _1), false),
    visualizer_("debug/dynup"),
    robot_model_loader_("robot_description", false),
    listener_(tf_buffer_) {

  robot_model_loader_.loadKinematicsSolvers(std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>());
  robot_model::RobotModelPtr kinematic_model = robot_model_loader_.getModel();
  if (!kinematic_model) {
    ROS_FATAL("No robot model loaded, killing dynup.");
    exit(1);
  }
  robot_state::RobotStatePtr init_state;
  init_state.reset(new robot_state::RobotState(kinematic_model));
  // set elbows to make arms straight, in a stupid way since moveit is annoying
  std::vector<std::string> names_vec = {"LElbow", "RElbow"};
  std::vector<double> pos_vec = {-M_PI/2, M_PI/2};
  init_state->setJointPositions(names_vec[0], &pos_vec[0]);
  init_state->setJointPositions(names_vec[1], &pos_vec[1]);
  init_state->updateLinkTransforms();
  // get shoulder and wrist pose
  geometry_msgs::Pose shoulder_origin, wrist_origin;
  tf2::convert(init_state->getGlobalLinkTransform("l_upper_arm"), shoulder_origin);
  tf2::convert(init_state->getGlobalLinkTransform("l_wrist"), wrist_origin);
  //compute arm length
  double arm_max_length = shoulder_origin.position.z - wrist_origin.position.z;
  //arm max length, y offset, z offset from base link
  engine_.init(arm_max_length, shoulder_origin.position.y, shoulder_origin.position.z);
  stabilizer_.setRobotModel(kinematic_model);
  ik_.init(kinematic_model);
  stabilizer_.init(kinematic_model);

  joint_goal_publisher_ = node_handle_.advertise<bitbots_msgs::JointCommand>("dynup_motor_goals", 1);
  debug_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("debug_markers", 1);
  cop_subscriber_ = node_handle_.subscribe("imu/data", 1, &DynUpNode::imuCallback, this);
  joint_state_subscriber_ = node_handle_.subscribe("joint_states", 1, &DynUpNode::jointStateCallback, this);
  server_.start();
}

void DynUpNode::jointStateCallback(const sensor_msgs::JointState &jointstates) {
    ik_.setCurrentJointStates(jointstates);
}

void DynUpNode::imuCallback(const sensor_msgs::Imu &msg) {
    stabilizer_.setImu(msg);
}

void DynUpNode::reconfigureCallback(bitbots_dynup::DynUpConfig &config, uint32_t level) {
  engine_rate_ = config.engine_rate;
  debug_ = config.display_debug;

  DynUpConfig params = config;

  engine_.setParams(params);

  stabilizer_.useStabilizing(config.stabilizing);

  ik_.useStabilizing(config.stabilizing);

  VisualizationParams viz_params = VisualizationParams();
  viz_params.spline_smoothness = config.spline_smoothness;
  visualizer_.setParams(viz_params);
}

void DynUpNode::executeCb(const bitbots_msgs::DynUpGoalConstPtr &goal) {
  // TODO: maybe switch to goal callback to be able to reject goals properly
  ROS_INFO("Accepted new goal");
  engine_.reset();
  ik_.reset();
  stabilizer_.reset();
  last_ros_update_time_ = 0;
  if (std::optional<std::tuple<geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose>> poses = getCurrentPoses()) {
    DynupRequest request;
    request.l_foot_pose = std::get<0>(poses.value());
    request.direction = goal->direction;
    ik_.setDirection(request.direction);
    request.r_foot_pose = std::get<1>(poses.value());
    request.l_hand_pose = std::get<2>(poses.value());
    request.r_hand_pose = std::get<3>(poses.value());
    engine_.setGoals(request);
    if(debug_) {
      visualizer_.displaySplines(engine_.getRFootSplines(), "base_link");
      visualizer_.displaySplines(engine_.getLFootSplines(), "r_sole");
      // Workaround for an error in the Visualizer. TODO
      if(request.direction == "front" || request.direction == "back") {
          visualizer_.displaySplines(engine_.getLHandSplines(), "base_link");
          visualizer_.displaySplines(engine_.getRHandSplines(), "base_link");
      }
    }
    ros::Rate loop_rate(engine_rate_);
    loopEngine(loop_rate);
    bitbots_msgs::DynUpResult r;
    if(server_.isPreemptRequested()){
      r.successful = false;
    }else{
      r.successful = true;
    }
    server_.setSucceeded(r);
  } else {
    ROS_ERROR("Could not determine positions! Aborting standup.");
    bitbots_msgs::DynUpResult r;
    r.successful = false;
    server_.setAborted(r);
  }
}

double DynUpNode::getTimeDelta() {
  // compute actual time delta that happened
  double dt;
  double current_ros_time = ros::Time::now().toSec();

  // first call needs to be handled specially
  if (last_ros_update_time_==0){
    last_ros_update_time_ = current_ros_time;
    return 0.001;
  }
  dt = current_ros_time - last_ros_update_time_;
  // this can happen due to floating point precision
  if (dt == 0) {
    ROS_WARN("dt was 0. this can happen in simulation if your update rate is higher than the simulators.");
    dt = 0.001;
  }
  last_ros_update_time_ = current_ros_time;
  return dt;
}

void DynUpNode::loopEngine(ros::Rate loop_rate) {
  int failed_tick_counter = 0;
  double dt;
  /* Do the loop as long as nothing cancels it */
  while (server_.isActive() && !server_.isPreemptRequested()) {
    dt = getTimeDelta();
    DynupResponse response = engine_.update(dt);
    stabilizer_.setRSoleToTrunk(tf_buffer_.lookupTransform("r_sole", "base_link", ros::Time(0)));
    DynupResponse stabilized_response = stabilizer_.stabilize(response, ros::Duration(dt));
    bitbots_splines::JointGoals goals = ik_.calculate(stabilized_response);
    bitbots_msgs::DynUpFeedback feedback;
    feedback.percent_done = engine_.getPercentDone();
    server_.publishFeedback(feedback);
    publishGoals(goals);
    if(goals.first.empty()) {
      failed_tick_counter++;
    }
    if (feedback.percent_done >= 100) {
      ROS_DEBUG("Completed dynup with %d failed ticks.", failed_tick_counter);
      break;
    }

    /* Let ROS do some important work of its own and sleep afterwards */
    ros::spinOnce();
    loop_rate.sleep();
  }
}

std::optional<std::tuple<geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose>> DynUpNode::getCurrentPoses() {
  ros::Time time = ros::Time::now();

  /* Construct zero-positions for all poses in their respective local frames */
  geometry_msgs::PoseStamped l_foot_origin, r_foot_origin, l_hand_origin, r_hand_origin;
  l_foot_origin.header.frame_id = "l_sole";
  l_foot_origin.pose.orientation.w = 1;
  l_foot_origin.header.stamp = time;

  r_foot_origin.header.frame_id = "r_sole";
  r_foot_origin.pose.orientation.w = 1;
  r_foot_origin.header.stamp = time;

  l_hand_origin.header.frame_id = "l_wrist";
  l_hand_origin.pose.orientation.w = 1;
  l_hand_origin.header.stamp = time;

  r_hand_origin.header.frame_id = "r_wrist";
  r_hand_origin.pose.orientation.w = 1;
  r_hand_origin.header.stamp = time;

  /* Transform the left foot into the right foot frame and all other splines into the base link frame*/
  geometry_msgs::PoseStamped l_foot_transformed, r_foot_transformed, l_hand_transformed, r_hand_transformed;
  try {
    //0.2 second timeout for transformations
    tf_buffer_.transform(l_foot_origin, l_foot_transformed, "r_sole", ros::Duration(0.2));
    tf_buffer_.transform(r_foot_origin, r_foot_transformed, "base_link", ros::Duration(0.2));
    tf_buffer_.transform(l_hand_origin, l_hand_transformed, "base_link", ros::Duration(0.2));
    tf_buffer_.transform(r_hand_origin, r_hand_transformed, "base_link", ros::Duration(0.2));
    return std::make_tuple(l_foot_transformed.pose, r_foot_transformed.pose, l_hand_transformed.pose, r_hand_transformed.pose);
  } catch (tf2::TransformException &exc) {
    ROS_ERROR_STREAM(exc.what());
    return std::nullopt;
  }

}

void DynUpNode::publishGoals(const bitbots_splines::JointGoals &goals) {
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

  joint_goal_publisher_.publish(command);
}

}

int main(int argc, char *argv[]) {
  /* Setup ROS node */
  ros::init(argc, argv, "dynup");
  bitbots_dynup::DynUpNode node;

  /* Setup dynamic_reconfigure */
  dynamic_reconfigure::Server<bitbots_dynup::DynUpConfig> dyn_reconf_server;
  dynamic_reconfigure::Server<bitbots_dynup::DynUpConfig>::CallbackType f;
  f = boost::bind(&bitbots_dynup::DynUpNode::reconfigureCallback, &node, _1, _2);
  dyn_reconf_server.setCallback(f);

  ROS_INFO("Initialized DynUp and waiting for actions");
  ros::spin();
}
