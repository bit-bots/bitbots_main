#include "bitbots_quintic_walk/quintic_walking_node.h"

namespace bitbots_quintic_walk {

QuinticWalkingNode::QuinticWalkingNode() :
    robot_model_loader_("/robot_description", false) {
  // init variables
  robot_state_ = humanoid_league_msgs::RobotControlState::CONTROLABLE;
  walk_engine_ = bitbots_quintic_walk::QuinticWalk();
  is_left_support_ = true; //TODO in engine?
  current_request_.orders = tf2::Transform();
  marker_id_ = 1;
  odom_broadcaster_ = tf2_ros::TransformBroadcaster();

  // read config
  nh_.param<double>("engine_frequency", engine_frequency_, 100.0);
  nh_.param<bool>("/simulation_active", simulation_active_, false);
  nh_.param<bool>("/walking/publishOdomTF", publish_odom_tf_, false);

  /* init publisher and subscriber */
  command_msg_ = bitbots_msgs::JointCommand();
  pub_controller_command_ = nh_.advertise<bitbots_msgs::JointCommand>("walking_motor_goals", 1);
  odom_msg_ = nav_msgs::Odometry();
  pub_odometry_ = nh_.advertise<nav_msgs::Odometry>("walk_odometry", 1);
  pub_support_ = nh_.advertise<std_msgs::Char>("walk_support_state", 1);
  sub_cmd_vel_ = nh_.subscribe("cmd_vel", 1, &QuinticWalkingNode::cmdVelCb, this,
                               ros::TransportHints().tcpNoDelay());
  sub_rob_state_ = nh_.subscribe("robot_state", 1, &QuinticWalkingNode::robStateCb, this,
                                 ros::TransportHints().tcpNoDelay());
  sub_joint_states_ =
      nh_.subscribe("joint_states", 1, &QuinticWalkingNode::jointStateCb, this, ros::TransportHints().tcpNoDelay());
  sub_kick_ = nh_.subscribe("kick", 1, &QuinticWalkingNode::kickCb, this, ros::TransportHints().tcpNoDelay());
  sub_imu_ = nh_.subscribe("imu/data", 1, &QuinticWalkingNode::imuCb, this, ros::TransportHints().tcpNoDelay());
  sub_pressure_ = nh_.subscribe("foot_pressure_filtered", 1, &QuinticWalkingNode::pressureCb, this,
                                ros::TransportHints().tcpNoDelay());
  sub_cop_l_ = nh_.subscribe("cop_l", 1, &QuinticWalkingNode::copLCb, this, ros::TransportHints().tcpNoDelay());
  sub_cop_r_ = nh_.subscribe("cop_r", 1, &QuinticWalkingNode::copRCb, this, ros::TransportHints().tcpNoDelay());


  /* debug publisher */
  pub_debug_ = nh_.advertise<bitbots_quintic_walk::WalkingDebug>("walk_debug", 1);
  pub_debug_marker_ = nh_.advertise<visualization_msgs::Marker>("walk_debug_marker", 1);

  //load MoveIt! model
  robot_model_loader_.loadKinematicsSolvers(
      kinematics_plugin_loader::KinematicsPluginLoaderPtr(
          new kinematics_plugin_loader::KinematicsPluginLoader()));
  kinematic_model_ = robot_model_loader_.getModel();
  all_joints_group_ = kinematic_model_->getJointModelGroup("All");
  legs_joints_group_ = kinematic_model_->getJointModelGroup("Legs");
  lleg_joints_group_ = kinematic_model_->getJointModelGroup("LeftLeg");
  rleg_joints_group_ = kinematic_model_->getJointModelGroup("RightLeg");
  goal_state_.reset(new robot_state::RobotState(kinematic_model_));
  goal_state_->setToDefaultValues();
  // we have to set some good initial position in the goal state, since we are using a gradient
  // based method. Otherwise, the first step will be not correct
  std::vector<std::string> names_vec = {"LHipPitch", "LKnee", "LAnklePitch", "RHipPitch", "RKnee", "RAnklePitch"};
  std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
  for (int i = 0; i < names_vec.size(); i++) {
    // besides its name, this method only changes a single joint position...
    goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
  }

  current_state_.reset(new robot_state::RobotState(kinematic_model_));
  current_state_->setToDefaultValues();

  // initilize IK solver
  bio_ik_solver_ = bitbots_ik::BioIKSolver(*all_joints_group_, *lleg_joints_group_, *rleg_joints_group_);
  bio_ik_solver_.set_use_approximate(true);

  first_run_ = true;

  // initialize dynamic-reconfigure
  server_.setCallback(boost::bind(&QuinticWalkingNode::reconfCallback, this, _1, _2));
}

void QuinticWalkingNode::run() {
  int odom_counter = 0;

  while (ros::ok()) {
    ros::Rate loop_rate(engine_frequency_);
    double dt = getTimeDelta();

    if (robot_state_==humanoid_league_msgs::RobotControlState::FALLING) {
      // the robot fell, we have to reset everything and do nothing else
      walk_engine_.reset();
    } else {
      // we don't want to walk, even if we have orders, if we are not in the right state
      /* Our robots will soon^TM be able to sit down and stand up autonomously, when sitting down the motors are
       * off but will turn on automatically which is why MOTOR_OFF is a valid walkable state. */
      // TODO Figure out a better way than having integration knowledge that HCM will play an animation to stand up
      current_request_.walkable_state = robot_state_==humanoid_league_msgs::RobotControlState::CONTROLABLE ||
          robot_state_==humanoid_league_msgs::RobotControlState::WALKING
          || robot_state_==humanoid_league_msgs::RobotControlState::MOTOR_OFF;
      // update walk engine response
      walk_engine_.setGoals(current_request_);
      WalkResponse response = walk_engine_.update(dt);
      // only calculate joint goals from this if the engine is not idle
      if (walk_engine_.getState()!="idle") { //todo
        calculateAndPublishJointGoals(response);
      }
    }

    // publish odometry
    odom_counter++;
    if (odom_counter > odom_pub_factor_) {
      publishOdometry();
      odom_counter = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void QuinticWalkingNode::calculateAndPublishJointGoals(WalkResponse response) {
  // change goals from support foot based coordinate system to trunk based coordinate system
  tf2::Transform trunk_to_support_foot_goal = response.support_foot_to_trunk.inverse();
  tf2::Transform trunk_to_flying_foot_goal = trunk_to_support_foot_goal*response.support_foot_to_flying_foot;

  // call ik solver
  bool success = bio_ik_solver_.solve(trunk_to_support_foot_goal, trunk_to_flying_foot_goal,
                                      walk_engine_.getFootstep().isLeftSupport(), goal_state_);

  // publish goals if successful
  if (success) {
    std::vector<std::string> joint_names = legs_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(legs_joints_group_, joint_goals);
    publishControllerCommands(joint_names, joint_goals);
  }

  // publish current support state
  std_msgs::Char support_state;
  if (walk_engine_.isDoubleSupport()) {
    support_state.data = 'd';
  } else if (walk_engine_.isLeftSupport()) {
    support_state.data = 'l';
  } else {
    support_state.data = 'r';
  }
  pub_support_.publish(support_state);

  // publish debug information
  if (debug_active_) {
    publishDebug(trunk_to_support_foot_goal, trunk_to_flying_foot_goal);
    publishMarkers();
  }
}

double QuinticWalkingNode::getTimeDelta() {
  // compute time delta depended if we are currently in simulation or reality
  double dt;
  double current_ros_time = ros::Time::now().toSec();
  dt = current_ros_time - last_ros_update_time_;
  if (dt==0) {
    ROS_WARN("dt was 0");
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

void QuinticWalkingNode::cmdVelCb(const geometry_msgs::Twist msg) {
  // we use only 3 values from the twist messages, as the robot is not capable of jumping or spinning around its
  // other axis.

  // the engine expects orders in [m] not [m/s]. We have to compute by dividing by step frequency which is a double step
  // factor 2 since the order distance is only for a single step, not double step
  double factor = (1.0/(params_.freq))/2.0;
  tf2::Vector3 orders = {msg.linear.x*factor, msg.linear.y*factor, msg.angular.z*factor};

  // the orders should not extend beyond a maximal step size
  for (int i = 0; i < 3; i++) {
    orders[i] = std::max(std::min(orders[i], max_step_[i]), max_step_[i]*-1);
  }
  // translational orders (x+y) should not exceed combined limit. scale if necessary
  if (max_step_xy_!=0) {
    double scaling_factor = (orders[0] + orders[1])/max_step_xy_;
    for (int i = 0; i < 2; i++) {
      orders[i] = orders[i]/std::max(scaling_factor, 1.0);
    }
  }

  // warn user that speed was limited
  if (msg.linear.x*factor!=orders[0] ||
      msg.linear.y*factor!=orders[1] ||
      msg.angular.z*factor!=orders[2]) {
    ROS_WARN(
        "Speed command was x: %.2f y: %.2f z: %.2f xy: %.2f but maximum is x: %.2f y: %.2f z: %.2f xy: %.2f",
        msg.linear.x, msg.linear.y, msg.angular.z, msg.linear.x + msg.linear.y, max_step_[0]/factor,
        max_step_[1]/factor, max_step_[2]/factor, max_step_xy_/factor);
  }

  // The result is the transform where the engine should place it next foot, we only use x,y and yaw
  current_request_.orders.setOrigin({orders[0], orders[1], 0});
  tf2::Quaternion quat;
  quat.setRPY(0, 0, orders[2]);
  current_request_.orders.setRotation(quat);
}

void QuinticWalkingNode::imuCb(const sensor_msgs::Imu msg) {
  if (imu_active_) {
    // the incoming geometry_msgs::Quaternion is transformed to a tf2::Quaterion
    tf2::Quaternion quat;
    tf2::convert(msg.orientation, quat);

    // the tf2::Quaternion has a method to access roll pitch and yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // compute the pitch offset to the currently wanted pitch of the engine
    double wanted_pitch =
        params_.trunk_pitch + params_.trunk_pitch_p_coef_forward*walk_engine_.getFootstep().getNextPos().x()
            + params_.trunk_pitch_p_coef_turn*fabs(walk_engine_.getFootstep().getNextPos().z());
    pitch = pitch + wanted_pitch;

    // get angular velocities
    double roll_vel = msg.angular_velocity.x;
    double pitch_vel = msg.angular_velocity.y;
    if (abs(roll) > imu_roll_threshold_ || abs(pitch) > imu_pitch_threshold_ ||
        abs(pitch_vel) > imu_pitch_vel_threshold_ || abs(roll_vel) > imu_roll_vel_threshold_) {
      walk_engine_.requestPause();
      if (abs(roll) > imu_roll_threshold_) {
        ROS_WARN("imu roll angle stop");
      } else if (abs(pitch) > imu_pitch_threshold_) {
        ROS_WARN("imu pitch angle stop");
      } else if (abs(pitch_vel) > imu_pitch_vel_threshold_) {
        ROS_WARN("imu roll vel stop");
      } else {
        ROS_WARN("imu pitch vel stop");
      }
    }
  }
}

void QuinticWalkingNode::pressureCb(
    const bitbots_msgs::FootPressure msg) { // TODO Remove this method since cop_cb is now used
  // we just want to look at the support foot. choose the 4 values from the message accordingly
  // s = support, n = not support, i = inside, o = outside, f = front, b = back
  double sob;
  double sof;
  double sif;
  double sib;

  double nob;
  double nof;
  double nif;
  double nib;

  if (walk_engine_.isLeftSupport()) {
    sob = msg.l_l_b;
    sof = msg.l_l_f;
    sif = msg.l_r_f;
    sib = msg.l_r_b;

    nib = msg.r_l_b;
    nif = msg.r_l_f;
    nof = msg.r_r_f;
    nob = msg.r_r_b;
  } else {
    sib = msg.r_l_b;
    sif = msg.r_l_f;
    sof = msg.r_r_f;
    sob = msg.r_r_b;

    nob = msg.l_l_b;
    nof = msg.l_l_f;
    nif = msg.l_r_f;
    nib = msg.l_r_b;
  }

  // sum to get overall pressure on not support foot
  double n_sum = nob + nof + nif + nib;

  // ratios between pressures to get relative position of CoP
  double s_io_ratio = 100;
  if (sof + sob!=0) {
    s_io_ratio = (sif + sib)/(sof + sob);
    if (s_io_ratio==0) {
      s_io_ratio = 100;
    }
  }
  double s_fb_ratio = 100;
  if (sib + sob!=0) {
    s_fb_ratio = (sif + sof)/(sib + sob);
    if (s_fb_ratio==0) {
      s_fb_ratio = 100;
    }
  }

  // check for early step end
  // phase has to be far enough (almost at end of step) to have right foot lifted
  // foot has to have ground contact
  double phase = walk_engine_.getPhase();
  if (phase_reset_active_ && ((phase > 0.5 - phase_reset_phase_ && phase < 0.5) || (phase > 1 - phase_reset_phase_)) &&
      n_sum > ground_min_pressure_) {
    ROS_WARN("Phase resetted!");
    walk_engine_.endStep();
  }

  // check if robot is unstable and should pause
  // this is true if the robot is falling to the outside or to front or back
  if (pressure_stop_active_ && (s_io_ratio > io_pressure_threshold_ || 1/s_io_ratio > io_pressure_threshold_ ||
      1/s_fb_ratio > fb_pressure_threshold_ || s_fb_ratio > fb_pressure_threshold_)) {
    walk_engine_.requestPause();

    //TODO this is debug
    if (s_io_ratio > io_pressure_threshold_ || 1/s_io_ratio > io_pressure_threshold_) {
      ROS_WARN("CoP io stop!");
    } else {
      ROS_WARN("CoP fb stop!");
    }
  }

  // decide which CoP
  geometry_msgs::PointStamped cop;
  if (walk_engine_.isLeftSupport()) {
    cop = cop_l_;
  } else {
    cop = cop_r_;
  }

  if (cop_stop_active_ && (abs(cop.point.x) > cop_x_threshold_ || abs(cop.point.y) > cop_y_threshold_)) {
    walk_engine_.requestPause();
    if (abs(cop.point.x) > cop_x_threshold_) {
      ROS_WARN("cop x stop");
    } else {
      ROS_WARN("cop y stop");
    }
  }

}

void QuinticWalkingNode::robStateCb(const humanoid_league_msgs::RobotControlState msg) {
  robot_state_ = msg.state;
}

void QuinticWalkingNode::jointStateCb(const sensor_msgs::JointState msg) {
  std::vector<std::string> names_vec = msg.name;
  std::string *names = names_vec.data();

  current_state_->setJointPositions(*names, msg.position.data());
}

void QuinticWalkingNode::kickCb(const std_msgs::BoolConstPtr msg) {
  walk_engine_.requestKick(msg->data);
}

void QuinticWalkingNode::copLCb(const geometry_msgs::PointStamped msg) {
  cop_l_ = msg;
}

void QuinticWalkingNode::copRCb(const geometry_msgs::PointStamped msg) {
  cop_r_ = msg;
}

void
QuinticWalkingNode::reconfCallback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config,
                                   uint32_t level) {
  params_ = config;

  walk_engine_.reconfCallback(params_);
  bio_ik_solver_.set_bioIK_timeout(config.bio_ik_time);

  debug_active_ = config.debug_active;
  engine_frequency_ = config.engine_freq;
  odom_pub_factor_ = config.odom_pub_factor;

  max_step_[0] = config.max_step_x;
  max_step_[1] = config.max_step_y;
  max_step_[2] = config.max_step_z;
  max_step_xy_ = config.max_step_xy;

  imu_active_ = config.imu_active;
  imu_pitch_threshold_ = config.imu_pitch_threshold;
  imu_roll_threshold_ = config.imu_roll_threshold;
  imu_pitch_vel_threshold_ = config.imu_pitch_vel_threshold;
  imu_roll_vel_threshold_ = config.imu_roll_vel_threshold;

  phase_reset_active_ = config.phase_reset_active;
  phase_reset_phase_ = config.phase_reset_phase;
  ground_min_pressure_ = config.ground_min_pressure;
  cop_stop_active_ = config.cop_stop_active;
  cop_x_threshold_ = config.cop_x_threshold;
  cop_y_threshold_ = config.cop_y_threshold;
  pressure_stop_active_ = config.pressure_stop_active;
  io_pressure_threshold_ = config.io_pressure_threshold;
  fb_pressure_threshold_ = config.fb_pressure_threshold;
  params_.pause_duration = config.pause_duration;
}

void
QuinticWalkingNode::publishControllerCommands(std::vector<std::string> joint_names, std::vector<double> positions) {
  // publishes the commands to the GroupedPositionController
  command_msg_.header.stamp = ros::Time::now();
  command_msg_.joint_names = joint_names;
  command_msg_.positions = positions;
  std::vector<double> ones(joint_names.size(), -1.0);
  std::vector<double> vels(joint_names.size(), -1.0);
  std::vector<double> accs(joint_names.size(), -1.0);
  std::vector<double> pwms(joint_names.size(), -1.0);
  command_msg_.velocities = vels;
  command_msg_.accelerations = accs;
  command_msg_.max_currents = pwms;

  pub_controller_command_.publish(command_msg_);
}

void QuinticWalkingNode::publishOdometry() {
  // transformation from support leg to trunk
  Eigen::Isometry3d trunk_to_support;
  if (walk_engine_.getFootstep().isLeftSupport()) {
    trunk_to_support = goal_state_->getGlobalLinkTransform("l_sole");
  } else {
    trunk_to_support = goal_state_->getGlobalLinkTransform("r_sole");
  }
  Eigen::Isometry3d support_to_trunk = trunk_to_support.inverse();
  tf2::Transform tf_support_to_trunk;
  tf2::convert(support_to_trunk, tf_support_to_trunk);

  // odometry to trunk is transform to support foot * transform from support to trunk
  tf2::Transform support_foot_tf;
  if (walk_engine_.getFootstep().isLeftSupport()) {
    support_foot_tf = walk_engine_.getFootstep().getLeft();
  } else {
    support_foot_tf = walk_engine_.getFootstep().getRight();
  }

  tf2::Transform odom_to_trunk = support_foot_tf*tf_support_to_trunk;
  tf2::Vector3 pos = odom_to_trunk.getOrigin();
  geometry_msgs::Quaternion quat_msg;

  tf2::convert(odom_to_trunk.getRotation().normalize(), quat_msg);

  ros::Time current_time = ros::Time::now();

  if (publish_odom_tf_) {
    odom_trans_ = geometry_msgs::TransformStamped();
    odom_trans_.header.stamp = current_time;
    odom_trans_.header.frame_id = "odom";
    odom_trans_.child_frame_id = "base_link";

    odom_trans_.transform.translation.x = pos[0];
    odom_trans_.transform.translation.y = pos[1];
    odom_trans_.transform.translation.z = pos[2];
    odom_trans_.transform.rotation = quat_msg;

    //send the transform
    odom_broadcaster_.sendTransform(odom_trans_);
  }

  // send the odometry also as message
  odom_msg_.header.stamp = current_time;
  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_link";
  odom_msg_.pose.pose.position.x = pos[0];
  odom_msg_.pose.pose.position.y = pos[1];
  odom_msg_.pose.pose.position.z = pos[2];

  odom_msg_.pose.pose.orientation = quat_msg;
  geometry_msgs::Twist twist;

  twist.linear.x = current_request_.orders.getOrigin()[0]*params_.freq*2;
  twist.linear.y = current_request_.orders.getOrigin()[1]*params_.freq*2;
  double roll, pitch, yaw;
  tf2::Matrix3x3(current_request_.orders.getRotation()).getRPY(roll, pitch, yaw);
  twist.angular.z = yaw*params_.freq*2;

  odom_msg_.twist.twist = twist;
  pub_odometry_.publish(odom_msg_);
}

void QuinticWalkingNode::publishDebug(tf2::Transform &trunk_to_support_foot_goal,
                                      tf2::Transform &trunk_to_flying_foot_goal) {
  /*
  This method publishes various debug / visualization information.
  */
  bitbots_quintic_walk::WalkingDebug msg;
  bool is_left_support = walk_engine_.isLeftSupport();
  msg.is_left_support = is_left_support;
  msg.is_double_support = walk_engine_.isDoubleSupport();
  msg.header.stamp = ros::Time::now();

  // define current support frame
  std::string current_support_frame;
  if (is_left_support) {
    current_support_frame = "l_sole";
  } else {
    current_support_frame = "r_sole";
  }

  // define colors based on current support state
  float r, g, b, a;
  if (walk_engine_.isDoubleSupport()) {
    r = 0;
    g = 0;
    b = 1;
    a = 1;
  } else if (walk_engine_.isLeftSupport()) {
    r = 1;
    g = 0;
    b = 0;
    a = 1;
  } else {
    r = 1;
    g = 1;
    b = 0;
    a = 1;
  }


  // times
  msg.phase_time = walk_engine_.getPhase();
  msg.traj_time = walk_engine_.getTrajsTime();

  msg.engine_state.data = walk_engine_.getState();

  // footsteps
  msg.footstep_last.x = walk_engine_.getFootstep().getLastPos()[0];
  msg.footstep_last.y = walk_engine_.getFootstep().getLastPos()[1];
  msg.footstep_last.z = walk_engine_.getFootstep().getLastEuler()[2];

  msg.footstep_next.x = walk_engine_.getFootstep().getNextPos()[0];
  msg.footstep_next.y = walk_engine_.getFootstep().getNextPos()[1];
  msg.footstep_next.z = walk_engine_.getFootstep().getNextEuler()[2];



  // engine output
  geometry_msgs::Pose pose_msg;
  tf2::convert(foot_pos_, pose_msg.position);
  tf2::Quaternion tf_quat;
  tf_quat.setRPY(foot_axis_[0], foot_axis_[1], foot_axis_[2]);
  tf2::convert(tf_quat, pose_msg.orientation);
  msg.engine_fly_goal = pose_msg;
  publishMarker("engine_fly_goal", current_support_frame, pose_msg, 0, 0, 1, a);

  msg.engine_fly_axis.x = foot_axis_[0];
  msg.engine_fly_axis.x = foot_axis_[1];
  msg.engine_fly_axis.x = foot_axis_[2];

  tf2::convert(trunk_pos_, pose_msg.position);
  tf_quat.setRPY(trunk_axis_[0], trunk_axis_[1], trunk_axis_[2]);
  tf2::convert(tf_quat, pose_msg.orientation);
  msg.engine_trunk_goal = pose_msg;
  publishMarker("engine_trunk_goal", current_support_frame, pose_msg, r, g, b, a);

  if (trunk_pos_[1] > 0) {
    trunk_pos_[1] = trunk_pos_[1] - params_.foot_distance/2;
  } else {
    trunk_pos_[1] = trunk_pos_[1] + params_.foot_distance/2;
  }
  tf2::convert(trunk_pos_, pose_msg.position);
  msg.engine_trunk_goal_abs = pose_msg;

  msg.engine_trunk_axis.x = trunk_axis_[0];
  msg.engine_trunk_axis.y = trunk_axis_[1];
  msg.engine_trunk_axis.z = trunk_axis_[2];

  // resulting trunk pose
  geometry_msgs::Pose pose;
  geometry_msgs::Point point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  pose.position = point;
  publishMarker("trunk_result", "base_link", pose, r, g, b, a);

  // goals
  geometry_msgs::Pose pose_support_foot_goal;
  tf2::toMsg(trunk_to_support_foot_goal, pose_support_foot_goal);
  msg.support_foot_goal = pose_support_foot_goal;
  geometry_msgs::Pose pose_fly_foot_goal;
  tf2::toMsg(trunk_to_flying_foot_goal, pose_fly_foot_goal);
  msg.fly_foot_goal = pose_fly_foot_goal;
  if (is_left_support) {
    msg.left_foot_goal = pose_support_foot_goal;
    msg.right_foot_goal = pose_fly_foot_goal;
  } else {
    msg.left_foot_goal = pose_fly_foot_goal;
    msg.right_foot_goal = pose_support_foot_goal;
  }
  publishMarker("engine_left_goal", "base_link", msg.left_foot_goal, 0, 1, 0, 1);
  publishMarker("engine_right_goal", "base_link", msg.right_foot_goal, 1, 0, 0, 1);

  // IK results
  geometry_msgs::Pose pose_left_result;
  tf2::convert(goal_state_->getGlobalLinkTransform("l_sole"), pose_left_result);
  msg.left_foot_ik_result = pose_left_result;
  geometry_msgs::Pose pose_right_result;
  tf2::convert(goal_state_->getGlobalLinkTransform("r_sole"), pose_right_result);
  msg.right_foot_ik_result = pose_right_result;
  if (is_left_support) {
    msg.support_foot_ik_result = pose_left_result;
    msg.fly_foot_ik_result = pose_right_result;
  } else {
    msg.support_foot_ik_result = pose_right_result;
    msg.fly_foot_ik_result = pose_left_result;
  }
  publishMarker("ik_left", "base_link", pose_left_result, 0, 1, 0, 1);
  publishMarker("ik_right", "base_link", pose_right_result, 1, 0, 0, 1);

  // IK offsets
  tf2::Vector3 support_off;
  tf2::Vector3 fly_off;
  tf2::Vector3 tf_vec_left;
  tf2::Vector3 tf_vec_right;
  Eigen::Vector3d l_transform = goal_state_->getGlobalLinkTransform("l_sole").translation();
  Eigen::Vector3d r_transform = goal_state_->getGlobalLinkTransform("r_sole").translation();
  tf2::convert(l_transform, tf_vec_left);
  tf2::convert(r_transform, tf_vec_right);
  geometry_msgs::Vector3 vect_msg;
  if (is_left_support) {
    support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_left;
    fly_off = trunk_to_flying_foot_goal.getOrigin() - tf_vec_right;
    tf2::convert(support_off, vect_msg);
    msg.left_foot_ik_offset = vect_msg;
    tf2::convert(fly_off, vect_msg);
    msg.right_foot_ik_offset = vect_msg;
  } else {
    support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_right;
    fly_off = trunk_to_flying_foot_goal.getOrigin() - tf_vec_left;
    tf2::convert(fly_off, vect_msg);
    msg.left_foot_ik_offset = vect_msg;
    tf2::convert(support_off, vect_msg);
    msg.right_foot_ik_offset = vect_msg;
  }
  tf2::convert(support_off, vect_msg);
  msg.support_foot_ik_offset = vect_msg;
  tf2::convert(fly_off, vect_msg);
  msg.fly_foot_ik_offset = vect_msg;

  // actual positions
  geometry_msgs::Pose pose_left_actual;
  tf2::convert(current_state_->getGlobalLinkTransform("l_sole"), pose_left_actual);
  msg.left_foot_position = pose_left_actual;
  geometry_msgs::Pose pose_right_actual;
  tf2::convert(current_state_->getGlobalLinkTransform("r_sole"), pose_right_actual);
  msg.right_foot_position = pose_right_actual;
  if (is_left_support) {
    msg.support_foot_position = pose_left_actual;
    msg.fly_foot_position = pose_right_actual;
  } else {
    msg.support_foot_position = pose_right_actual;
    msg.fly_foot_position = pose_left_actual;
  }

  // actual offsets
  l_transform = current_state_->getGlobalLinkTransform("l_sole").translation();
  r_transform = current_state_->getGlobalLinkTransform("r_sole").translation();
  tf2::convert(l_transform, tf_vec_left);
  tf2::convert(r_transform, tf_vec_right);
  if (is_left_support) {
    support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_left;
    fly_off = trunk_to_flying_foot_goal.getOrigin() - tf_vec_right;
    tf2::convert(support_off, vect_msg);
    msg.left_foot_actual_offset = vect_msg;
    tf2::convert(fly_off, vect_msg);
    msg.right_foot_actual_offset = vect_msg;
  } else {
    support_off = trunk_to_support_foot_goal.getOrigin() - tf_vec_right;
    fly_off = trunk_to_flying_foot_goal.getOrigin() - tf_vec_left;
    tf2::convert(fly_off, vect_msg);
    msg.left_foot_actual_offset = vect_msg;
    tf2::convert(support_off, vect_msg);
    msg.right_foot_actual_offset = vect_msg;
  }
  tf2::convert(support_off, vect_msg);
  msg.support_foot_actual_offset = vect_msg;
  tf2::convert(fly_off, vect_msg);
  msg.fly_foot_actual_offset = vect_msg;

  pub_debug_.publish(msg);
}

void QuinticWalkingNode::publishMarker(std::string name_space,
                                       std::string frame,
                                       geometry_msgs::Pose pose, float r, float g, float b, float a) {
  visualization_msgs::Marker marker_msg;
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.header.frame_id = frame;

  marker_msg.type = marker_msg.ARROW;
  marker_msg.ns = name_space;
  marker_msg.action = marker_msg.ADD;
  marker_msg.pose = pose;

  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  marker_msg.color = color;

  geometry_msgs::Vector3 scale;
  scale.x = 0.01;
  scale.y = 0.003;
  scale.z = 0.003;
  marker_msg.scale = scale;

  marker_msg.id = marker_id_;
  marker_id_++;

  pub_debug_marker_.publish(marker_msg);
}

void QuinticWalkingNode::publishMarkers() {
  //publish markers
  visualization_msgs::Marker marker_msg;
  marker_msg.header.stamp = ros::Time::now();
  if (walk_engine_.getFootstep().isLeftSupport()) {
    marker_msg.header.frame_id = "l_sole";
  } else {
    marker_msg.header.frame_id = "r_sole";
  }
  marker_msg.type = marker_msg.CUBE;
  marker_msg.action = 0;
  marker_msg.lifetime = ros::Duration(0.0);
  geometry_msgs::Vector3 scale;
  scale.x = 0.20;
  scale.y = 0.10;
  scale.z = 0.01;
  marker_msg.scale = scale;
  //last step
  marker_msg.ns = "last_step";
  marker_msg.id = 1;
  std_msgs::ColorRGBA color;
  color.r = 0;
  color.g = 0;
  color.b = 0;
  color.a = 1;
  marker_msg.color = color;
  geometry_msgs::Pose pose;
  tf2::Vector3 step_pos = walk_engine_.getFootstep().getLastPos();
  geometry_msgs::Point point;
  point.x = step_pos[0];
  point.y = step_pos[1];
  point.z = 0;
  pose.position = point;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, step_pos[2]);
  tf2::convert(q, pose.orientation);
  marker_msg.pose = pose;
  pub_debug_marker_.publish(marker_msg);

  //last step center
  marker_msg.ns = "step_center";
  marker_msg.id = marker_id_;
  scale.x = 0.01;
  scale.y = 0.01;
  scale.z = 0.01;
  marker_msg.scale = scale;
  pub_debug_marker_.publish(marker_msg);

  // next step
  marker_msg.id = marker_id_;
  marker_msg.ns = "next_step";
  scale.x = 0.20;
  scale.y = 0.10;
  scale.z = 0.01;
  marker_msg.scale = scale;
  color.r = 1;
  color.g = 1;
  color.b = 1;
  color.a = 0.5;
  marker_msg.color = color;
  step_pos = walk_engine_.getFootstep().getNextPos();
  point.x = step_pos[0];
  point.y = step_pos[1];
  pose.position = point;
  q.setRPY(0.0, 0.0, step_pos[2]);
  tf2::convert(q, pose.orientation);
  marker_msg.pose = pose;
  pub_debug_marker_.publish(marker_msg);

  marker_id_++;
}

void QuinticWalkingNode::initializeEngine() {
  walk_engine_.reset();
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "quintic_walking");
  // init node
  bitbots_quintic_walk::QuinticWalkingNode node;

  // run the node
  node.initializeEngine();
  node.run();
}
