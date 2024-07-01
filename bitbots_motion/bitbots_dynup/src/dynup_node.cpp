#include "bitbots_dynup/dynup_node.hpp"

namespace bitbots_dynup {
using namespace std::chrono_literals;

DynupNode::DynupNode(rclcpp::Node::SharedPtr node, const std::string &ns, std::vector<rclcpp::Parameter> parameters)
    : node_(node),
      joint_goal_publisher_(node_->create_publisher<bitbots_msgs::msg::JointCommand>("dynup_motor_goals", 1)),
      imu_subscriber_(node_->create_subscription<sensor_msgs::msg::Imu>("imu/data", 1,
                                                                        std::bind(&DynupNode::imuCallback, this, _1))),
      param_listener_(node_),
      params_(param_listener_.get_params()),
      engine_(node_, params_.engine),
      stabilizer_(node_, params_.stabilizer),
      visualizer_(node_, params_.visualizer, "debug/dynup"),
      ik_(node_),
      tf_buffer_(node_->get_clock()),
      tf_listener_(tf_buffer_, node_) {
  // We need to create a new node for moveit, otherwise dynamic reconfigure will be broken...
  auto moveit_node = std::make_shared<rclcpp::Node>(ns + "dynup_moveit_node");

  // when called from python, parameters are given to the constructor
  for (auto parameter : parameters) {
    if (node_->has_parameter(parameter.get_name())) {
      // this is the case for dynup engine params set via python
      node_->set_parameter(parameter);
    } else {
      // parameter is not for the walking, set on moveit node
      moveit_node->declare_parameter(parameter.get_name(), parameter.get_type());
      moveit_node->set_parameter(parameter);
    }
  }
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
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 10 * 1e3,
                           "Can't copy parameters from move_group node. Service not available, waiting again...");
    }
    rcl_interfaces::msg::ListParametersResult parameter_list =
        parameters_client->list_parameters({"robot_description_kinematics"}, 10);
    auto copied_parameters = parameters_client->get_parameters(parameter_list.names);
    for (auto &parameter : copied_parameters) {
      moveit_node->declare_parameter(parameter.get_name(), parameter.get_type());
      moveit_node->set_parameter(parameter);
    }
  }

  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(moveit_node, "robot_description");
  kinematic_model_ = robot_model_loader_->getModel();
  if (!kinematic_model_) {
    RCLCPP_FATAL(node_->get_logger(), "No robot model loaded, killing dynup.");
    exit(1);
  }

  // load params once
  onSetParameters();

  moveit::core::RobotStatePtr init_state = std::make_shared<moveit::core::RobotState>(kinematic_model_);
  // set elbows to make arms straight, in a stupid way since moveit is annoying
  std::vector<std::string> names_vec = {"LElbow", "RElbow"};
  std::vector<double> pos_vec{-M_PI / 2, M_PI / 2};
  init_state->setJointPositions(names_vec[0], &pos_vec[0]);
  init_state->setJointPositions(names_vec[1], &pos_vec[1]);
  init_state->updateLinkTransforms();
  // get shoulder and wrist pose
  geometry_msgs::msg::Pose shoulder_origin;
  tf2::convert(init_state->getGlobalLinkTransform("l_upper_arm"), shoulder_origin);
  // arm max length, y offset, z offset from base link
  engine_.init(shoulder_origin.position.y, shoulder_origin.position.z);
  ik_.init(kinematic_model_);

  action_server_ = rclcpp_action::create_server<DynupGoal>(node_, "dynup", std::bind(&DynupNode::goalCb, this, _1, _2),
                                                           std::bind(&DynupNode::cancelCb, this, _1),
                                                           std::bind(&DynupNode::acceptedCb, this, _1));

  RCLCPP_INFO(node_->get_logger(), "Initialized DynUp and waiting for actions");
}

bitbots_msgs::msg::JointCommand DynupNode::step(double dt, const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  // method for python interface. take all messages as parameters instead of using ROS
  imuCallback(imu_msg);
  // update dynup engine response
  bitbots_msgs::msg::JointCommand joint_goals = step(dt);
  return joint_goals;
}

bitbots_msgs::msg::JointCommand DynupNode::step(double dt) {
  if (dt <= 0) {
    dt = 0.001;
  }

  // Run the engine
  DynupResponse response = engine_.update(dt);

  // Apply the stabilizer
  stabilizer_.setRSoleToTrunk(
      tf_buffer_.lookupTransform(params_.node.tf.r_sole_frame, params_.node.tf.base_link_frame, rclcpp::Time(0)));
  DynupResponse stabilized_response = stabilizer_.stabilize(response, rclcpp::Duration::from_nanoseconds(1e9 * dt));

  // Calculate the joint goals (IK)
  bitbots_splines::JointGoals goals = ik_.calculate(stabilized_response);

  visualizer_.publishIKOffsets(kinematic_model_, stabilized_response, goals);

  // Check if we found a solution
  if (goals.first.empty()) {
    failed_tick_counter_++;
  }

  // Check if we are stable as determined by the stabilizer
  if (stabilizer_.isStable()) {
    stable_duration_ += 1;
  } else {
    stable_duration_ = 0;
  }

  // Build goal message that will be sent to the motor controller
  return createGoalMsg(goals);
}

geometry_msgs::msg::PoseArray DynupNode::step_open_loop(double dt) {
  // Calculate goal poses for the next step, but don't do any IK
  DynupNode::step(dt);
  bitbots_dynup::msg::DynupPoses pose_msg = DynupNode::getCurrentPoses();
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.poses = {pose_msg.l_leg_pose, pose_msg.r_leg_pose, pose_msg.l_arm_pose, pose_msg.r_arm_pose};
  return pose_array;
}

void DynupNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) { stabilizer_.setImu(msg); }

void DynupNode::onSetParameters() {
  engine_.setParams(params_.engine);
  stabilizer_.setParams(params_.stabilizer);
  visualizer_.setParams(params_.visualizer);
}

void DynupNode::reset(int time) {
  engine_.reset(time);
  ik_.reset();
  stabilizer_.reset();
}

void DynupNode::execute(const std::shared_ptr<DynupGoalHandle> goal_handle) {
  RCLCPP_INFO(node_->get_logger(), "Dynup accepted new goal");
  const auto goal = goal_handle->get_goal();
  reset();
  last_ros_update_time_ = 0;
  start_time_ = node_->get_clock()->now().seconds();

  if (param_listener_.is_old(params_)) {
    params_ = param_listener_.get_params();
    // Copy all params to other modules
    onSetParameters();
  }

  bitbots_utils::wait_for_tf(
      node_->get_logger(), node_->get_clock(), &tf_buffer_,
      {params_.node.tf.base_link_frame, params_.node.tf.r_sole_frame, params_.node.tf.l_sole_frame,
       params_.node.tf.r_wrist_frame, params_.node.tf.l_wrist_frame},
      params_.node.tf.base_link_frame);

  bitbots_dynup::msg::DynupPoses poses = getCurrentPoses();
  if (poses.header.stamp.nanosec != 0) {
    DynupRequest request;
    request.direction = getDynupDirection(goal->direction);
    ik_.setDirection(request.direction);
    request.l_foot_pose = poses.l_leg_pose;
    request.r_foot_pose = poses.r_leg_pose;
    request.l_hand_pose = poses.l_arm_pose;
    request.r_hand_pose = poses.r_arm_pose;
    engine_.setGoals(request);
    if (params_.visualizer.display_debug) {
      visualizer_.displaySplines(engine_.getRFootSplines(), params_.node.tf.base_link_frame);
      visualizer_.displaySplines(engine_.getLFootSplines(), params_.node.tf.r_sole_frame);
      // Workaround for an error in the Visualizer. TODO
      if (request.direction == DynupDirection::FRONT || request.direction == DynupDirection::BACK) {
        visualizer_.displaySplines(engine_.getLHandSplines(), params_.node.tf.base_link_frame);
        visualizer_.displaySplines(engine_.getRHandSplines(), params_.node.tf.base_link_frame);
      }
    }
    loopEngine(params_.engine.engine_rate, goal_handle);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Could not determine positions! Aborting standup.");
    bitbots_msgs::action::Dynup_Result::SharedPtr r = std::make_shared<bitbots_msgs::action::Dynup_Result>();
    r->successful = false;
    server_free_ = true;
    goal_handle->abort(r);
  }
}

rclcpp_action::GoalResponse DynupNode::goalCb(const rclcpp_action::GoalUUID &uuid,
                                              std::shared_ptr<const DynupGoal::Goal> goal) {
  RCLCPP_INFO(node_->get_logger(), "Received goal request");
  (void)uuid;
  if (server_free_) {
    server_free_ = false;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Dynup is busy, goal rejected!");
    return rclcpp_action::GoalResponse::REJECT;
  }
}

rclcpp_action::CancelResponse DynupNode::cancelCb(const std::shared_ptr<DynupGoalHandle> goal) {
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
  (void)goal;
  server_free_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DynupNode::acceptedCb(const std::shared_ptr<DynupGoalHandle> goal) {
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DynupNode::execute, this, _1), goal}.detach();
}

double DynupNode::getTimeDelta() {
  // compute actual time delta that happened
  double current_ros_time = node_->get_clock()->now().seconds();

  // first call needs to be handled specially
  if (last_ros_update_time_ == 0) {
    last_ros_update_time_ = current_ros_time;
    return 0.001;
  }
  double dt = current_ros_time - last_ros_update_time_;
  // this can happen due to floating point precision or simulation issues. will be catched later
  if (dt == 0) {
    RCLCPP_WARN_ONCE(node_->get_logger(),
                     "dt was 0. this can happen in simulation if your update rate is higher than the simulators. This "
                     "warning is only displayed once!");
  }
  last_ros_update_time_ = current_ros_time;
  return dt;
}

void DynupNode::loopEngine(int loop_rate, std::shared_ptr<DynupGoalHandle> goal_handle) {
  auto result = std::make_shared<DynupGoal::Result>();
  failed_tick_counter_ = 0;
  bitbots_msgs::msg::JointCommand msg;
  /* Do the loop as long as nothing cancels it */
  while (rclcpp::ok()) {
    rclcpp::Time startTime = node_->get_clock()->now();
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(node_->get_logger(), "Goal canceled");
      return;
    }
    msg = step(getTimeDelta());
    auto feedback = std::make_shared<bitbots_msgs::action::Dynup_Feedback>();
    feedback->percent_done = engine_.getPercentDone();
    goal_handle->publish_feedback(feedback);
    if (feedback->percent_done >= 100 &&
        // Check if we are stable as determined by the stabilizer (we finished the end pause)
        (stable_duration_ >= params_.stabilizer.end_pause.duration || !(params_.stabilizer.end_pause.active) ||
         (node_->get_clock()->now().seconds() - start_time_ >=
          engine_.getDuration() + params_.stabilizer.end_pause.timeout))) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Completed dynup with " << failed_tick_counter_ << " failed ticks.");
      result->successful = true;
      server_free_ = true;
      goal_handle->succeed(result);
      return;
    }
    if (msg.joint_names.empty()) {
      continue;
    }
    joint_goal_publisher_->publish(msg);
    node_->get_clock()->sleep_until(startTime + rclcpp::Duration::from_nanoseconds(1e9 / loop_rate));
  }
}

bitbots_dynup::msg::DynupPoses DynupNode::getCurrentPoses() {
  rclcpp::Time time = node_->get_clock()->now();
  /* Transform the left foot into the right foot frame and all other splines into the base link frame*/
  bitbots_dynup::msg::DynupPoses msg;
  try {
    // Timeout for transformations
    auto timeout = tf2::durationFromSec(1.0);

    // Shorten the names
    auto tf_names = params_.node.tf;

    // Get the transforms of the end effectors
    geometry_msgs::msg::Transform l_foot_transformed =
        tf_buffer_.lookupTransform(tf_names.r_sole_frame, tf_names.l_sole_frame, time, timeout).transform;
    geometry_msgs::msg::Transform r_foot_transformed =
        tf_buffer_.lookupTransform(tf_names.base_link_frame, tf_names.r_sole_frame, time, timeout).transform;
    geometry_msgs::msg::Transform l_hand_transformed =
        tf_buffer_.lookupTransform(tf_names.base_link_frame, tf_names.l_wrist_frame, time, timeout).transform;
    geometry_msgs::msg::Transform r_hand_transformed =
        tf_buffer_.lookupTransform(tf_names.base_link_frame, tf_names.r_wrist_frame, time, timeout).transform;

    std::function transform2pose = [](geometry_msgs::msg::Transform transform) {
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
    msg.header.stamp = node_->get_clock()->now();
    return msg;
  } catch (tf2::TransformException &exc) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), exc.what());
    return msg;
  }
}

bitbots_msgs::msg::JointCommand DynupNode::createGoalMsg(const bitbots_splines::JointGoals &goals) {
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
  command.velocities = std::vector<double>(goals.first.size(), -1.0);
  command.accelerations = std::vector<double>(goals.first.size(), -1.0);
  command.max_currents = std::vector<double>(goals.first.size(), -1.0);

  return command;
}

DynupEngine *DynupNode::getEngine() { return &engine_; }

DynupIK *DynupNode::getIK() { return &ik_; }

}  // namespace bitbots_dynup

int main(int argc, char **argv) {
  // init node
  rclcpp::init(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("dynup");

  // Create dynup
  [[maybe_unused]] bitbots_dynup::DynupNode dynup(node);

  // Create executor
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  // Spin executor to process callbacks
  exec.spin();
  rclcpp::shutdown();
}
