#include "humanoid_league_team_communication/team_communication.h"

using namespace robocup;
using namespace humanoid;

TeamCommunication::TeamCommunication() : nh_(), transform_listener_(tf_buffer_) {
  // --- Params ---
  ros::NodeHandle pnh("~");
  int team;
  nh_.getParam("team_id", team);
  nh_.getParam("bot_id", player_);

  int port;
  pnh.getParam("port", port);
  //publishing rate in Hz
  pnh.getParam("rate", frequency_);
  pnh.getParam("avg_walking_speed", avg_walking_speed_);
  int teamcolor;
  pnh.getParam("team_color", teamcolor);
  team_color_ = static_cast<Team>(teamcolor);
  pnh.getParam("lifetime", lifetime_);
  pnh.getParam("belief_threshold", belief_threshold_);

  pnh.getParam("team_data", teamdata_topic_);
  pnh.getParam("strategy", strategy_topic_);
  pnh.getParam("robot_state", robot_state_topic_);
  pnh.getParam("position", position_topic_);
  pnh.getParam("ball", ball_topic_);
  pnh.getParam("obstacles", obstacles_topic_);

  std::fill(*position_cov_, *position_cov_ + 3 * 3, 0);

  // --- Init UDP Connection ---
  udp_connection_ = new UdpConnection(port);

  // --- Initialize Topics ---
  publisher_ = nh_.advertise<humanoid_league_msgs::TeamData>(teamdata_topic_, 10);

  sub_role_ = nh_.subscribe(strategy_topic_, 1, &TeamCommunication::strategyCallback, this,
                            ros::TransportHints().tcpNoDelay());
  sub_robot_state_ = nh_.subscribe(robot_state_topic_, 1, &TeamCommunication::robotStateCallback,
                                   this, ros::TransportHints().tcpNoDelay());
  sub_position_ = nh_.subscribe(position_topic_, 1, &TeamCommunication::positionCallback, this,
                                ros::TransportHints().tcpNoDelay());
  sub_ball_ = nh_.subscribe(ball_topic_, 1, &TeamCommunication::ballsCallback, this,
                            ros::TransportHints().tcpNoDelay());
  sub_obstacles_ = nh_.subscribe(obstacles_topic_, 1, &TeamCommunication::obstaclesCallback,
                                 this, ros::TransportHints().tcpNoDelay());
}

void TeamCommunication::run() {
  // TODO: std::thread nutzen?
  pthread_t thread;
  pthread_create(&thread, nullptr, TeamCommunication::startRecvThread, this);
  timer_ = nh_.createTimer(ros::Duration(1.0f / frequency_), &TeamCommunication::sendThread, this);
}

void *TeamCommunication::startRecvThread(void *context) {
  ((TeamCommunication *) context)->recvThread();
  return nullptr;
}

void TeamCommunication::recvThread() {
  ros::Rate thread_rate(TeamCommunication::frequency_);
  while (ros::ok()) {
    Message recv_msg;
    recv_msg = udp_connection_->receive_data();
    // do not publish the robot's own data
    if (int(recv_msg.current_pose().player_id()) != player_) {
      publishData(recv_msg);
    }
    thread_rate.sleep();
  }
}

void TeamCommunication::sendThread(const ros::TimerEvent &) {
  if (state_ != PENALISED) {
    Message send_msg;

    // set timestamp
    ros::Time time = ros::Time::now();
    ::google::protobuf::Timestamp *proto_timestamp = new ::google::protobuf::Timestamp();
    proto_timestamp->set_seconds(time.sec);
    proto_timestamp->set_nanos(time.nsec);
    send_msg.set_allocated_timestamp(proto_timestamp);

    // set state
    send_msg.set_state(state_);

    // set strategy
    send_msg.set_role(role_);
    send_msg.set_offensive_side(offensive_side_);
    send_msg.set_action(action_);

    // set position
    if (position_belief_ >= belief_threshold_ && ros::Time::now().toSec() - position_exists_ < lifetime_) {
      Robot *current_pose = new Robot();
      current_pose->set_player_id(player_);
      fvec3 *pose_position = new fvec3();
      pose_position->set_x(position_x_);
      pose_position->set_y(position_y_);
      pose_position->set_z(position_orientation_);
      current_pose->set_allocated_position(pose_position);
      fmat3 *pose_covariance = new fmat3();
      fvec3 *pose_covariance_x = new fvec3();
      fvec3 *pose_covariance_y = new fvec3();
      fvec3 *pose_covariance_z = new fvec3();
      pose_covariance_x->set_x(position_cov_[0][0]);
      pose_covariance_x->set_y(position_cov_[0][1]);
      pose_covariance_x->set_z(position_cov_[0][2]);
      pose_covariance_y->set_x(position_cov_[1][0]);
      pose_covariance_y->set_y(position_cov_[1][1]);
      pose_covariance_y->set_z(position_cov_[1][2]);
      pose_covariance_z->set_x(position_cov_[2][0]);
      pose_covariance_z->set_y(position_cov_[2][1]);
      pose_covariance_z->set_z(position_cov_[2][2]);
      pose_covariance->set_allocated_x(pose_covariance_x);
      pose_covariance->set_allocated_y(pose_covariance_y);
      pose_covariance->set_allocated_z(pose_covariance_z);
      current_pose->set_allocated_covariance(pose_covariance);
      current_pose->set_team(team_color_);
      send_msg.set_allocated_current_pose(current_pose);
      send_msg.set_position_confidence(position_belief_);
    }

    // set ball
    if (ball_belief_ >= belief_threshold_ && ros::Time::now().toSec() - ball_exists_ < lifetime_) {
      Ball *ball = new Ball();
      fvec3 *ball_position = new fvec3();
      ball_position->set_x(ball_relative_x_);
      ball_position->set_y(ball_relative_y_);
      ball->set_allocated_position(ball_position);
      send_msg.set_allocated_ball(ball);
      send_msg.set_ball_confidence(ball_belief_);
      send_msg.set_time_to_ball(time_to_position_at_ball_);
    }

    // set obstacles
    if (ros::Time::now().toSec() - obstacles_exists_ < lifetime_) {
      Robot *current_obstacle;
      for (auto const &obstacle : obstacles_) {
        if (obstacle.belief >= belief_threshold_) {
          current_obstacle = send_msg.add_others();
          current_obstacle->set_player_id(obstacle.player_number);
          fvec3 *obstacle_pos = new fvec3();
          obstacle_pos->set_x(obstacle.x);
          obstacle_pos->set_y(obstacle.y);
          current_obstacle->set_allocated_position(obstacle_pos);
          current_obstacle->set_team(obstacle.teamcolor);
          send_msg.add_obstacle_confidence(obstacle.belief);
        }
      }
    }
    // broadcast data
    udp_connection_->send_data(send_msg);
  }
}

void TeamCommunication::publishData(Message received_msg) {
  // build message
  humanoid_league_msgs::TeamData message;

  // timestamp
  message.header.stamp.sec = google::protobuf::util::TimeUtil::TimestampToSeconds(received_msg.timestamp());
  message.header.stamp.nsec =
      google::protobuf::util::TimeUtil::TimestampToNanoseconds(received_msg.timestamp()) % int(pow(10, 9));

  // set frame of the pose parameters
  message.header.frame_id = "map";

  // robot_id
  message.robot_id = received_msg.current_pose().player_id();

  // state
  message.state = received_msg.state();

  // strategy
  message.strategy.role = received_msg.role();
  message.strategy.action = received_msg.action();
  message.strategy.offensive_side = received_msg.offensive_side();

  // robot_position
  message.robot_position.pose.pose.position.x = received_msg.current_pose().position().x();
  message.robot_position.pose.pose.position.y = received_msg.current_pose().position().y();
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, received_msg.current_pose().position().z());
  tf2::convert(quaternion, message.robot_position.pose.pose.orientation);
  message.robot_position.pose.covariance[0] = received_msg.current_pose().covariance().x().x();
  message.robot_position.pose.covariance[1] = received_msg.current_pose().covariance().x().y();
  message.robot_position.pose.covariance[5] = received_msg.current_pose().covariance().x().z();
  message.robot_position.pose.covariance[6] = received_msg.current_pose().covariance().y().x();
  message.robot_position.pose.covariance[7] = received_msg.current_pose().covariance().y().y();
  message.robot_position.pose.covariance[11] = received_msg.current_pose().covariance().y().z();
  message.robot_position.pose.covariance[30] = received_msg.current_pose().covariance().z().x();
  message.robot_position.pose.covariance[31] = received_msg.current_pose().covariance().z().y();
  message.robot_position.pose.covariance[35] = received_msg.current_pose().covariance().z().z();
  message.robot_position.confidence = received_msg.position_confidence();

  // ball, time_to_position_at_ball
  message.ball_relative.pose.pose.position.x = received_msg.ball().position().x();
  message.ball_relative.pose.pose.position.y = received_msg.ball().position().y();
  message.ball_relative.confidence = received_msg.ball_confidence();
  message.time_to_position_at_ball = received_msg.time_to_ball();

  // obstacles
  humanoid_league_msgs::ObstacleRelativeArray obstacles;
  obstacles.header.frame_id = "map";
  int num_of_obstacles = received_msg.others_size();
  for (int i = 0; i < num_of_obstacles; i++) {
    Robot recv_obstacle = received_msg.others(i);
    humanoid_league_msgs::ObstacleRelative obstacle;
    obstacle.pose.pose.pose.position.x = recv_obstacle.position().x();
    obstacle.pose.pose.pose.position.y = recv_obstacle.position().y();
    if (recv_obstacle.team() == RED) {
      obstacle.type = humanoid_league_msgs::ObstacleRelative::ROBOT_MAGENTA;
    } else if (recv_obstacle.team() == BLUE) {
      obstacle.type = humanoid_league_msgs::ObstacleRelative::ROBOT_CYAN;
    } else if (recv_obstacle.team() == UNKNOWN_TEAM) {
      obstacle.type = humanoid_league_msgs::ObstacleRelative::ROBOT_UNDEFINED;
    }
    obstacle.playerNumber = recv_obstacle.player_id();
    obstacle.pose.confidence = received_msg.obstacle_confidence(i);
    obstacles.obstacles.push_back(obstacle);
  }
  message.obstacles = obstacles;

  // publish data
  publisher_.publish(message);
}

void TeamCommunication::strategyCallback(humanoid_league_msgs::Strategy msg) {
  role_ = static_cast<Role>(msg.role);
  action_ = static_cast<Action>(msg.action);
  offensive_side_ = static_cast<OffensiveSide>(msg.offensive_side);
  strategy_exists_ = ros::Time::now().toSec();
}

void TeamCommunication::robotStateCallback(humanoid_league_msgs::RobotControlState msg) {
  uint8_t state = msg.state;
  // states in which the robot is penalized by the game controller
  if (state == humanoid_league_msgs::RobotControlState::PENALTY ||
      state == humanoid_league_msgs::RobotControlState::PENALTY_ANIMATION ||
      state == humanoid_league_msgs::RobotControlState::PICKED_UP) {
    state_ = PENALISED;
  }
    // state in which the robot is not able to play
  else if (state == humanoid_league_msgs::RobotControlState::STARTUP ||
      state == humanoid_league_msgs::RobotControlState::SHUTDOWN ||
      state == humanoid_league_msgs::RobotControlState::RECORD ||
      state == humanoid_league_msgs::RobotControlState::HCM_OFF ||
      state == humanoid_league_msgs::RobotControlState::HARDWARE_PROBLEM) {
    state_ = UNKNOWN_STATE;
  } else {
    state_ = UNPENALISED;
  }
}

void TeamCommunication::positionCallback(humanoid_league_msgs::PoseWithCertainty msg) {
  // transform robot pose and cov matrix to the map frame
  geometry_msgs::TransformStamped transform;
  geometry_msgs::PoseWithCovariance position_map;
  try {
    transform = tf_buffer_.lookupTransform("map", "base_footprint", ros::Time(0));
    tf2::doTransform(msg.pose.pose, position_map.pose, transform);
    tf2::Stamped<tf2::Transform> tf_transform;
    tf2::fromMsg(transform, tf_transform);
    position_map.covariance = tf2::transformCovariance(msg.pose.covariance, tf_transform);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("TeamComm: Robot pose is not send due to a transformation error: %s", ex.what());
    // If the robot pose could not be transformed, it should not be saved and send to the other robots which expect the
    // robot's pose to be in the map frame.
    return;
  }

  // get position
  position_x_ = position_map.pose.position.x;
  position_y_ = position_map.pose.position.y;

  // get orientation
  tf2::Quaternion tf2_quaternion;
  tf2::convert(position_map.pose.orientation, tf2_quaternion);
  position_orientation_ = static_cast<float>(tf2::getYaw(tf2_quaternion));

  // get belief
  position_cov_[0][0] = position_map.covariance[0];
  position_cov_[0][1] = position_map.covariance[1];
  position_cov_[0][2] = position_map.covariance[5];
  position_cov_[1][0] = position_map.covariance[6];
  position_cov_[1][1] = position_map.covariance[7];
  position_cov_[1][2] = position_map.covariance[11];
  position_cov_[2][0] = position_map.covariance[30];
  position_cov_[2][1] = position_map.covariance[31];
  position_cov_[2][2] = position_map.covariance[35];

  position_belief_ = msg.confidence;

  // set time to decide if the information is up to date when broadcasting it
  position_exists_ = ros::Time::now().toSec();
}

void TeamCommunication::ballsCallback(humanoid_league_msgs::PoseWithCertaintyArray msg) {
  // TODO: Replace sorting by just filtering for the ball with highest confidence
  auto sortByConfidence = [](humanoid_league_msgs::PoseWithCertainty &ball1,
                             humanoid_league_msgs::PoseWithCertainty &ball2) -> bool {
    return ball1.confidence >= ball2.confidence;
  };

  std::sort(msg.poses.begin(), msg.poses.end(), sortByConfidence);  // Sort balls by confidence
  humanoid_league_msgs::PoseWithCertainty ball = msg.poses[0];  // Choose ball with highest confidence

  // transform position to map frame
  geometry_msgs::TransformStamped transform;
  geometry_msgs::PoseWithCovariance ball_map;
  try {
    transform = tf_buffer_.lookupTransform("map", msg.header.frame_id, ros::Time(0));
    tf2::doTransform(ball.pose.pose, ball_map.pose, transform);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("TeamComm: Ball is not send due to a transformation error: %s", ex.what());
    // If the ball could not be transformed, it should not be saved and send to the other robots which expect the ball
    // to be in the map frame.
    return;
  }

  // get position
  ball_relative_x_ = ball_map.pose.position.x;
  ball_relative_y_ = ball_map.pose.position.y;

  // get belief
  ball_belief_ = ball.confidence;

  //use pythagoras to compute time to ball
  time_to_position_at_ball_ = sqrt((pow(ball_relative_x_, 2.0) + pow(ball_relative_y_, 2.0))) / avg_walking_speed_;

  // set time to decide if the information is up to date when broadcasting it
  ball_exists_ = msg.header.stamp.toSec();
}

void TeamCommunication::obstaclesCallback(const humanoid_league_msgs::ObstacleRelativeArray &msg) {
  // clear obstacle array because of new data
  obstacles_.clear();

  // get data of all recognized robots
  ObstacleData obstacle_data;

  for (auto const &obstacle : msg.obstacles) {
    // only robots are broadcasted
    if (obstacle.type != humanoid_league_msgs::ObstacleRelative::HUMAN
        && obstacle.type != humanoid_league_msgs::ObstacleRelative::POLE) {
      // reset values
      obstacle_data = {};

      // transform position to map frame
      geometry_msgs::TransformStamped transform;
      geometry_msgs::PoseWithCovariance obstacle_map;
      try {
        transform = tf_buffer_.lookupTransform("map", msg.header.frame_id, ros::Time(0));
        tf2::doTransform(obstacle.pose.pose.pose, obstacle_map.pose, transform);
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("TeamComm: Obstacle is not send due to a transformation error: %s", ex.what());
        // If the obstacle could not be transformed, it should not be saved and send to the other robots which expect
        // the obstacle to be in the map frame.
        continue;
      }
      // get position
      obstacle_data.x = obstacle_map.pose.position.x;
      obstacle_data.y = obstacle_map.pose.position.y;

      // get confidence
      obstacle_data.belief = obstacle.pose.confidence;

      // get player number
      obstacle_data.player_number = obstacle.playerNumber;

      // get team color
      if (obstacle.type == humanoid_league_msgs::ObstacleRelative::ROBOT_MAGENTA) {
        obstacle_data.teamcolor = RED;
      } else if (obstacle.type == humanoid_league_msgs::ObstacleRelative::ROBOT_CYAN) {
        obstacle_data.teamcolor = BLUE;
      } else {
        obstacle_data.teamcolor = UNKNOWN_TEAM;
      }

      // store one recognized robot
      obstacles_.push_back(obstacle_data);
    }
  }

  // set time to decide if the information is up to date when broadcasting it
  obstacles_exists_ = ros::Time::now().toSec();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "humanoid_league_team_communication");
  ROS_INFO("Starting Team Communication");
  // init node
  TeamCommunication node;
  // run the node
  node.run();
  ros::spin();
}
