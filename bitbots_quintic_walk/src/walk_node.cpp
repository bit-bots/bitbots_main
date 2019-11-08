#include "bitbots_quintic_walk/walk_node.h"

#include <memory>

namespace bitbots_quintic_walk {

WalkNode::WalkNode() :
    robot_model_loader_("/robot_description", false) {
  // init variables
  robot_state_ = humanoid_league_msgs::RobotControlState::CONTROLABLE;
  current_request_.orders = {0, 0, 0};
  current_trunk_pitch_ = 0;

  // read config
  nh_.param<double>("engine_frequency", engine_frequency_, 100.0);
  nh_.param<bool>("/simulation_active", simulation_active_, false);
  nh_.param<bool>("/walking/node/publish_odom_tf", publish_odom_tf_, false);

  /* init publisher and subscriber */
  pub_controller_command_ = nh_.advertise<bitbots_msgs::JointCommand>("walking_motor_goals", 1);
  pub_odometry_ = nh_.advertise<nav_msgs::Odometry>("walk_odometry", 1);
  pub_support_ = nh_.advertise<std_msgs::Char>("walk_support_state", 1);
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &WalkNode::cmdVelCb, this,
                               ros::TransportHints().tcpNoDelay());
  robot_state_sub_ = nh_.subscribe("robot_state", 1, &WalkNode::robStateCb, this,
                                   ros::TransportHints().tcpNoDelay());
  joint_state_sub_ =
      nh_.subscribe("joint_states", 1, &WalkNode::jointStateCb, this, ros::TransportHints().tcpNoDelay());
  kick_sub_ = nh_.subscribe("kick", 1, &WalkNode::kickCb, this, ros::TransportHints().tcpNoDelay());
  imu_sub_ = nh_.subscribe("imu/data", 1, &WalkNode::imuCb, this, ros::TransportHints().tcpNoDelay());
  pressure_sub_left_ = nh_.subscribe("foot_pressure_left/filtered", 1, &WalkNode::pressureLeftCb, this,
                                     ros::TransportHints().tcpNoDelay());
  pressure_sub_right_ = nh_.subscribe("foot_pressure_right/filtered", 1, &WalkNode::pressureRightCb, this,
                                      ros::TransportHints().tcpNoDelay());
  cop_l_sub_ = nh_.subscribe("cop_l", 1, &WalkNode::copLeftCb, this, ros::TransportHints().tcpNoDelay());
  cop_r_sub_ = nh_.subscribe("cop_r", 1, &WalkNode::copRightCb, this, ros::TransportHints().tcpNoDelay());


  //load MoveIt! model
  robot_model_loader_.loadKinematicsSolvers(std::make_shared<kinematics_plugin_loader::KinematicsPluginLoader>());
  kinematic_model_ = robot_model_loader_.getModel();
  if (!kinematic_model_) {
    ROS_FATAL("No robot model loaded, killing quintic walk.");
    exit(1);
  }
  //stabilizer_.setRobotModel(kinematic_model_);
  ik_.init(kinematic_model_);
  visualizer_.init(kinematic_model_);

  current_state_.reset(new robot_state::RobotState(kinematic_model_));
  current_state_->setToDefaultValues();

  first_run_ = true;

  // initialize dynamic-reconfigure
  dyn_reconf_server_ =
      new dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig>(ros::NodeHandle(
          "~/node"));
  dynamic_reconfigure::Server<bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig>::CallbackType f;
  f = boost::bind(&bitbots_quintic_walk::WalkNode::reconfCallback, this, _1, _2);
  dyn_reconf_server_->setCallback(f);

}

void WalkNode::run() {
  int odom_counter = 0;
  WalkResponse response;

  while (ros::ok()) {
    ros::Rate loop_rate(engine_frequency_);
    double dt = getTimeDelta();

    if (robot_state_ == humanoid_league_msgs::RobotControlState::FALLING) {
      // the robot fell, we have to reset everything and do nothing else
      walk_engine_.reset();
    } else {
      // we don't want to walk, even if we have orders, if we are not in the right state
      /* Our robots will soon^TM be able to sit down and stand up autonomously, when sitting down the motors are
       * off but will turn on automatically which is why MOTOR_OFF is a valid walkable state. */
      // TODO Figure out a better way than having integration knowledge that HCM will play an animation to stand up
      current_request_.walkable_state = robot_state_ == humanoid_league_msgs::RobotControlState::CONTROLABLE ||
          robot_state_ == humanoid_league_msgs::RobotControlState::WALKING ||
          robot_state_ == humanoid_league_msgs::RobotControlState::MOTOR_OFF;
      // update walk engine response
      walk_engine_.setGoals(current_request_);
      response = walk_engine_.update(dt);
      visualizer_.publishEngineDebug(response);

      // only calculate joint goals from this if the engine is not idle
      if (walk_engine_.getState() != WalkState::IDLE) {
        calculateAndPublishJointGoals(response, dt);
      }
    }

    // publish odometry
    odom_counter++;
    if (odom_counter > odom_pub_factor_) {
      publishOdometry(response);
      odom_counter = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void WalkNode::calculateAndPublishJointGoals(const WalkResponse &response, double dt) {
  // get bioIk goals from stabilizer
  std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals = stabilizer_.stabilize(response, current_trunk_pitch_, ros::Duration(dt));

  // compute motor goals from IK
  bitbots_splines::JointGoals motor_goals = ik_.calculate(std::move(ik_goals));

  // publish them
  publishGoals(motor_goals);

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
    visualizer_.publishIKDebug(response, current_state_, motor_goals);
    visualizer_.publishWalkMarkers(response);
  }
}

double WalkNode::getTimeDelta() {
  // compute time delta depended if we are currently in simulation or reality
  double dt;
  double current_ros_time = ros::Time::now().toSec();
  dt = current_ros_time - last_ros_update_time_;
  if (dt == 0) {
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

void WalkNode::cmdVelCb(const geometry_msgs::Twist msg) {
  // we use only 3 values from the twist messages, as the robot is not capable of jumping or spinning around its
  // other axis.

  // the engine expects orders in [m] not [m/s]. We have to compute by dividing by step frequency which is a double step
  // factor 2 since the order distance is only for a single step, not double step
  double factor = (1.0 / (walk_engine_.getFreq())) / 2.0;
  current_request_.orders = {msg.linear.x * factor, msg.linear.y * factor, msg.angular.z * factor};

  // the orders should not extend beyond a maximal step size
  for (int i = 0; i < 3; i++) {
    current_request_.orders[i] = std::max(std::min(current_request_.orders[i], max_step_[i]), max_step_[i] * -1);
  }
  // translational orders (x+y) should not exceed combined limit. scale if necessary
  if (max_step_xy_ != 0) {
    double scaling_factor = (current_request_.orders[0] + current_request_.orders[1]) / max_step_xy_;
    for (int i = 0; i < 2; i++) {
      current_request_.orders[i] = current_request_.orders[i] / std::max(scaling_factor, 1.0);
    }
  }

  // warn user that speed was limited
  if (msg.linear.x * factor != current_request_.orders[0] ||
      msg.linear.y * factor != current_request_.orders[1] ||
      msg.angular.z * factor != current_request_.orders[2]) {
    ROS_WARN(
        "Speed command was x: %.2f y: %.2f z: %.2f xy: %.2f but maximum is x: %.2f y: %.2f z: %.2f xy: %.2f",
        msg.linear.x, msg.linear.y, msg.angular.z, msg.linear.x + msg.linear.y, max_step_[0] / factor,
        max_step_[1] / factor, max_step_[2] / factor, max_step_xy_ / factor);
  }
}

void WalkNode::imuCb(const sensor_msgs::Imu &msg) {
  if (imu_active_) {
    // the incoming geometry_msgs::Quaternion is transformed to a tf2::Quaternion
    tf2::Quaternion quat;
    tf2::convert(msg.orientation, quat);

    // the tf2::Quaternion has a method to access roll pitch and yaw
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_trunk_pitch_ = pitch;

    // compute the pitch offset to the currently wanted pitch of the engine
    double wanted_pitch = walk_engine_.getWantedTrunkPitch();

    double pitch_delta = pitch - wanted_pitch;

    // get angular velocities
    double roll_vel = msg.angular_velocity.x;
    double pitch_vel = msg.angular_velocity.y;
    if (abs(roll) > imu_roll_threshold_ || abs(pitch_delta) > imu_pitch_threshold_ ||
        abs(pitch_vel) > imu_pitch_vel_threshold_ || abs(roll_vel) > imu_roll_vel_threshold_) {
      walk_engine_.requestPause();
      if (abs(roll) > imu_roll_threshold_) {
        ROS_WARN("imu roll angle stop");
      } else if (abs(pitch_delta) > imu_pitch_threshold_) {
        ROS_WARN("imu pitch angle stop");
      } else if (abs(pitch_vel) > imu_pitch_vel_threshold_) {
        ROS_WARN("imu roll vel stop");
      } else {
        ROS_WARN("imu pitch vel stop");
      }
    }
  }
}

void WalkNode::pressureLeftCb(const bitbots_msgs::FootPressure msg) {
  // only check if this foot is not the current support foot
  if (!walk_engine_.isLeftSupport()) {
    checkPhaseReset(msg);
  }
}

void WalkNode::pressureRightCb(const bitbots_msgs::FootPressure msg) {
  // only check if this foot is not the current support foot
  if (walk_engine_.isLeftSupport()) {
    checkPhaseReset(msg);
  }
}

void WalkNode::checkPhaseReset(bitbots_msgs::FootPressure msg) {
  /**
   * This method checks, if the foot made contact to the ground and ends step earlier, by resetting the phase.
   */
  double pressure_sum = msg.left_back + msg.left_front + msg.right_back + msg.right_front;

  // phase has to be far enough (almost at end of step) to have right foot lifted foot has to have ground contact
  double phase = walk_engine_.getPhase();
  if (phase_reset_active_ && ((phase > 0.5 - phase_reset_phase_ && phase < 0.5) || (phase > 1 - phase_reset_phase_)) &&
      pressure_sum > ground_min_pressure_) {
    ROS_WARN("Phase resetted!");
    walk_engine_.endStep();
  }
}

void WalkNode::copLeftCb(geometry_msgs::PointStamped msg) {
  // Since CoP is only published as something else than 0 if the foot is on the ground, we don't have to check
  // if this is the current support foot, or if it is double support.
  // The CoP should not go to the outside edge of the foot. This is the left side for the left foot (y is positive).
  // The inside edge is okay, since it is necessary to shift the CoM dynamically between feet.
  // To prevent falling to the front and back, the x position of the CoP is also taken into account.
  if (cop_stop_active_ &&
      (msg.point.y > cop_y_threshold_ || abs(msg.point.x) > cop_x_threshold_)) {
    walk_engine_.requestPause();
  }
}

void WalkNode::copRightCb(geometry_msgs::PointStamped msg) {
  // The CoP should not go to the outside edge of the foot. This is the right side for the right foot (y is negative).
  if (cop_stop_active_ &&
      (msg.point.y < -1 * cop_y_threshold_ || abs(msg.point.x) > cop_x_threshold_)) {
    walk_engine_.requestPause();
  }
}

void WalkNode::robStateCb(const humanoid_league_msgs::RobotControlState msg) {
  robot_state_ = msg.state;
}

void WalkNode::jointStateCb(const sensor_msgs::JointState &msg) {
  std::vector<std::string> names_vec = msg.name;
  std::string *names = names_vec.data();

  current_state_->setJointPositions(*names, msg.position.data());
}

void WalkNode::kickCb(const std_msgs::BoolConstPtr &msg) {
  walk_engine_.requestKick(msg->data);
}

void WalkNode::reconfCallback(bitbots_quintic_walk::bitbots_quintic_walk_paramsConfig &config, uint32_t level) {
  params_ = config;

  ik_.setBioIKTimeout(config.bio_ik_time);

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
  params_.pause_duration = config.pause_duration;
  walk_engine_.setPauseDuration(params_.pause_duration);
}

// todo this is the same method as in kick, maybe put it into a utility class - Yes - still needs to be discussed
// currently the spline package is quiet ros agnostic, this would not work with this method
void WalkNode::publishGoals(const bitbots_splines::JointGoals &goals) {
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

  pub_controller_command_.publish(command);
}

void WalkNode::publishOdometry(WalkResponse response) {
  // odometry to trunk is transform to support foot * transform from support to trunk
  tf2::Transform support_foot_tf;
  if (walk_engine_.isLeftSupport()) {
    support_foot_tf = walk_engine_.getLeft();
  } else {
    support_foot_tf = walk_engine_.getRight();
  }

  tf2::Transform odom_to_trunk = support_foot_tf * response.support_foot_to_trunk;
  tf2::Vector3 pos = odom_to_trunk.getOrigin();
  geometry_msgs::Quaternion quat_msg;
  tf2::convert(odom_to_trunk.getRotation().normalize(), quat_msg);

  ros::Time current_time = ros::Time::now();

  // send the odometry as message
  odom_msg_.header.stamp = current_time;
  odom_msg_.header.frame_id = "odom";
  odom_msg_.child_frame_id = "base_link";
  odom_msg_.pose.pose.position.x = pos[0];
  odom_msg_.pose.pose.position.y = pos[1];
  odom_msg_.pose.pose.position.z = pos[2];

  odom_msg_.pose.pose.orientation = quat_msg;
  geometry_msgs::Twist twist;

  twist.linear.x = current_request_.orders.x() * walk_engine_.getFreq() * 2;
  twist.linear.y = current_request_.orders.y() * walk_engine_.getFreq() * 2;
  twist.angular.z = current_request_.orders.z() * walk_engine_.getFreq() * 2;

  odom_msg_.twist.twist = twist;
  pub_odometry_.publish(odom_msg_);

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
}

void WalkNode::initializeEngine() {
  walk_engine_.reset();
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "quintic_walking");
  // init node
  bitbots_quintic_walk::WalkNode node;

  // run the node
  node.initializeEngine();
  node.run();
}
