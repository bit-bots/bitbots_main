#include "humanoid_league_team_communication/team_communication.h"

using namespace robocup;
using namespace humanoid;

TeamCommunication::TeamCommunication() : nh_(), transform_listener_(tf_buffer_) {
  // --- Params ---
  ros::NodeHandle pnh("~");
  int team;
  int player;
  nh_.getParam("team_id", team);
  nh_.getParam("bot_id", player);

  int port;
  pnh.getParam("port", port);
  //publishing rate in Hz
  pnh.getParam("rate", frequency_);
  pnh.getParam("avg_walking_speed", avg_walking_speed_);
  pnh.getParam("max_kicking_distance", max_kicking_distance_);
  int teamcolor;
  pnh.getParam("team_color", teamcolor);
  team_color_ = teamcolor;
  pnh.getParam("lifetime", lifetime_);
  pnh.getParam("belief_threshold", belief_threshold_);

  pnh.getParam("team_data", teamdata_topic_);
  pnh.getParam("strategy", strategy_topic_);
  pnh.getParam("robot_state", robot_state_topic_);
  pnh.getParam("goal", goal_topic_);
  pnh.getParam("position", position_topic_);
  pnh.getParam("ball", ball_topic_);
  pnh.getParam("obstacles", obstacles_topic_);

  std::fill(*position_cov_, *position_cov_ + 3*3, 0);

  // --- Init UDP Connection ---
  udp_connection_ = new UdpConnection(port);

  // --- Initialize Topics ---
  publisher_ = nh_.advertise<humanoid_league_msgs::TeamData>(teamdata_topic_, 10);

  sub_role_ = nh_.subscribe(strategy_topic_, 1, &TeamCommunication::strategyCallback, this,
                            ros::TransportHints().tcpNoDelay());
  sub_robot_state_ = nh_.subscribe(robot_state_topic_, 1, &TeamCommunication::robotStateCallback,
                                   this, ros::TransportHints().tcpNoDelay());
  sub_goal_ = nh_.subscribe(goal_topic_, 1, &TeamCommunication::goalCallback, this,
                            ros::TransportHints().tcpNoDelay());
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
}

void TeamCommunication::recvThread() {
  ros::Rate thread_rate(TeamCommunication::frequency_);
  while (ros::ok()) {
    Message *recv_msg;
    recv_msg = udp_connection_->receive_data();
    publishData(recv_msg);
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

    //    //state
    //    mitecom_.set_state(state_);
    //    mitecom_.set_action(action_);
    //    mitecom_.set_role(role_);
    //
    //    mitecom_.set_max_kicking_distance(max_kicking_distance_);
    //    mitecom_.set_get_avg_walking_speed(avg_walking_speed_);
    //
    //    //position
    //    if (position_belief_ > belief_threshold_ && ros::Time::now().sec - position_exists_ < lifetime_) {
    //      mitecom_.set_pos(position_x_, position_y_, position_orientation_, position_belief_);
    //    }
    //    //ball
    //    if (ball_belief_ > belief_threshold_ && ros::Time::now().sec - ball_exists_ < lifetime_) {
    //      mitecom_.set_relative_ball(ball_relative_x_, ball_relative_y_, ball_belief_);
    //    }
    //    else{
    //      // workaround to be able to identify in TeamData Msg which robot has sent the useful data
    //      mitecom_.set_relative_ball(1000.0, 1000.0, 0.0);
    //    }
    //    /*//opponent goal
    //    if (oppgoal_belief_ > 0) {
    //        mitecom_.set_opp_goal_relative(oppgoal_relative_x_, oppgoal_relative_y_, oppgoal_belief_);
    //    }*/
    //    if(ros::Time::now().sec - obstacles_exists_ < lifetime_) {
    //      //opponent robots
    //      if (!opponent_robots_.empty()) {
    //        mitecom_.set_opponent_robot_a(opponent_robots_[0][0], opponent_robots_[0][1], opponent_robots_[0][2]);
    //      }
    //      else{
    //        // workaround to be able to identify in TeamData Msg which robot has sent the useful data
    //        mitecom_.set_opponent_robot_a(1000.0, 1000.0, 0.0);
    //      }
    //      if (opponent_robots_.size() > 1) {
    //        mitecom_.set_opponent_robot_b(opponent_robots_[1][0], opponent_robots_[1][1], opponent_robots_[1][2]);
    //      }
    //      else{
    //        mitecom_.set_opponent_robot_b(1000.0, 1000.0, 0.0);
    //      }
    //      if (opponent_robots_.size() > 2) {
    //        mitecom_.set_opponent_robot_c(opponent_robots_[2][0], opponent_robots_[2][1], opponent_robots_[2][2]);
    //      }
    //      else{
    //        mitecom_.set_opponent_robot_c(1000.0, 1000.0, 0.0);
    //      }
    //      if (opponent_robots_.size() > 3) {
    //        mitecom_.set_opponent_robot_d(opponent_robots_[3][0], opponent_robots_[3][1], opponent_robots_[3][2]);
    //      }
    //      else{
    //        mitecom_.set_opponent_robot_d(1000.0, 1000.0, 0.0);
    //      }
    //
    //      //team robots
    //      if (!team_robots_.empty()) {
    //        mitecom_.set_team_robot_a(team_robots_[0][0], team_robots_[0][1], team_robots_[0][2]);
    //      }
    //      else{
    //        mitecom_.set_team_robot_a(1000.0, 1000.0, 0.0);
    //      }
    //      if (team_robots_.size() > 1) {
    //        mitecom_.set_team_robot_b(team_robots_[1][0], team_robots_[1][1], team_robots_[1][2]);
    //      }
    //      else{
    //        mitecom_.set_team_robot_b(1000.0, 1000.0, 0.0);
    //      }
    //      if (team_robots_.size() > 2) {
    //        mitecom_.set_team_robot_c(team_robots_[2][0], team_robots_[2][1], team_robots_[2][2]);
    //      }
    //      else{
    //        mitecom_.set_team_robot_c(1000.0, 1000.0, 0.0);
    //      }
    //    }
    //    else{
    //      // workaround to be able to identify in TeamData Msg which robot has sent the useful data
    //      mitecom_.set_opponent_robot_a(1000.0, 1000.0, 0.0);
    //      mitecom_.set_opponent_robot_b(1000.0, 1000.0, 0.0);
    //      mitecom_.set_opponent_robot_c(1000.0, 1000.0, 0.0);
    //      mitecom_.set_opponent_robot_d(1000.0, 1000.0, 0.0);
    //      mitecom_.set_team_robot_a(1000.0, 1000.0, 0.0);
    //      mitecom_.set_team_robot_b(1000.0, 1000.0, 0.0);
    //      mitecom_.set_team_robot_c(1000.0, 1000.0, 0.0);
    //
    //    }
    //
    //    //time to ball
    //    if(time_to_position_at_ball_set_ && ros::Time::now().sec - ball_exists_ < lifetime_) {
    //      mitecom_.set_time_to_ball(time_to_position_at_ball_);
    //    }
    //    // strategy
    //    if(offensive_side_set_) {
    //      mitecom_.set_offensive_side(offensive_side_);
    //    }
    udp_connection_->send_data(send_msg);
  }
}

void TeamCommunication::publishData(Message* received_msg){
//  std::vector<uint8_t> ids;
//  std::vector<uint8_t> roles;
//  std::vector<uint8_t> actions;
//  std::vector<uint8_t> states;
//  std::vector<geometry_msgs::Pose> own_position;
//  //std::vector<uint8_t> own_position_beliefs;   unnecessary because of TeamData.msg
//  std::vector<geometry_msgs::PoseWithCovariance> ball_relative;
//  //std::vector<humanoid_league_msgs::Position2D> oppgoal_relative;
//  std::vector<humanoid_league_msgs::ObstacleRelative> opponent_robot_a;
//  std::vector<humanoid_league_msgs::ObstacleRelative> opponent_robot_b;
//  std::vector<humanoid_league_msgs::ObstacleRelative> opponent_robot_c;
//  std::vector<humanoid_league_msgs::ObstacleRelative> opponent_robot_d;
//  std::vector<humanoid_league_msgs::ObstacleRelative> team_robot_a;
//  std::vector<humanoid_league_msgs::ObstacleRelative> team_robot_b;
//  std::vector<humanoid_league_msgs::ObstacleRelative> team_robot_c;
//  std::vector<std::vector<humanoid_league_msgs::ObstacleRelative>> obstacles;
//  std::vector<float> avg_walking_speeds;
//  std::vector<float> time_to_position_at_balls;
//  std::vector<float> max_kicking_distances;
//  std::vector<uint8_t> offensive_side;
//
//  //iterate through all robots from which we received data
//  for (auto const& x : team_data) {
//    MiTeCom::TeamMateData rob_data;
//    rob_data = *x.second;
//
//    ids.push_back(rob_data.get_id());
//    roles.push_back(rob_data.get_role());
//    actions.push_back(rob_data.get_action());
//    states.push_back(rob_data.get_state());
//
//    //own position
//    //TODO position confidence -> msg definitions
//    geometry_msgs::Pose pos_msg;
//    pos_msg.position.x = rob_data.get_absolute_x() / 1000.0;
//    pos_msg.position.y = rob_data.get_absolute_y() / 1000.0;
//    //pos_msg.theta = rob_data.get_absolute_orientation() / 1000.0; TODO this is a quaternion now
//    own_position.push_back(pos_msg);
//    //own_position_beliefs.push_back(rob_data.get_absolute_belief() / 255.0);   unnecessary because of TeamData.msg
//
//    //ball
//    geometry_msgs::PoseWithCovariance ball_msg;
//    ball_msg.pose.position.x = rob_data.get_relative_ball_x() / 1000.0;
//    ball_msg.pose.position.y = rob_data.get_relative_ball_y() / 1000.0;
//    // ball_msg.confidence = rob_data.get_ball_belief() / 255.0; TODO Conversion needed
//    ball_relative.push_back(ball_msg);
//
//    /*//oppgoal
//    humanoid_league_msgs::Position2D oppgoal_msg;
//    oppgoal_msg.pose.x = rob_data.get_oppgoal_relative_x() / 1000.0;
//    oppgoal_msg.pose.y = rob_data.get_oppgoal_relative_y() / 1000.0;
//    oppgoal_msg.confidence = rob_data.get_oppgoal_belief() / 255.0;
//    oppgoal_relative.push_back(oppgoal_msg);*/
//
//    //opponent_robot_a
//    humanoid_league_msgs::ObstacleRelative opponent_robot_a_msg;
//    opponent_robot_a_msg.pose.pose.pose.position.x = rob_data.get_opponent_robot_a_x() / 1000.0;
//    opponent_robot_a_msg.pose.pose.pose.position.y = rob_data.get_opponent_robot_a_y() / 1000.0;
//    //opponent_robot_a_msg.confidence = rob_data.get_opponent_robot_a_belief() / 255.0;
//    opponent_robot_a.push_back(opponent_robot_a_msg);
//
//    //opponent_robot_b
//    humanoid_league_msgs::ObstacleRelative opponent_robot_b_msg;
//    opponent_robot_b_msg.pose.pose.pose.position.x = rob_data.get_opponent_robot_b_x() / 1000.0;
//    opponent_robot_b_msg.pose.pose.pose.position.y = rob_data.get_opponent_robot_b_y() / 1000.0;
//    //opponent_robot_b_msg.confidence = rob_data.get_opponent_robot_b_belief() / 255.0;
//    opponent_robot_b.push_back(opponent_robot_b_msg);
//
//    //opponent_robot_c
//    humanoid_league_msgs::ObstacleRelative opponent_robot_c_msg;
//    opponent_robot_c_msg.pose.pose.pose.position.x = rob_data.get_opponent_robot_c_x() / 1000.0;
//    opponent_robot_c_msg.pose.pose.pose.position.y = rob_data.get_opponent_robot_c_y() / 1000.0;
//    //opponent_robot_c_msg.confidence = rob_data.get_opponent_robot_c_belief() / 255.0;
//    opponent_robot_c.push_back(opponent_robot_c_msg);
//
//    //opponent_robot_d
//    humanoid_league_msgs::ObstacleRelative opponent_robot_d_msg;
//    opponent_robot_d_msg.pose.pose.pose.position.x = rob_data.get_opponent_robot_d_x() / 1000.0;
//    opponent_robot_d_msg.pose.pose.pose.position.y = rob_data.get_opponent_robot_d_y() / 1000.0;
//    //opponent_robot_d_msg.confidence = rob_data.get_opponent_robot_d_belief() / 255.0;
//    opponent_robot_d.push_back(opponent_robot_d_msg);
//
//    //team_robot_a
//    humanoid_league_msgs::ObstacleRelative team_robot_a_msg;
//    team_robot_a_msg.pose.pose.pose.position.x = rob_data.get_team_robot_a_x() / 1000.0;
//    team_robot_a_msg.pose.pose.pose.position.y = rob_data.get_team_robot_a_y() / 1000.0;
//    //team_robot_a_msg.confidence = rob_data.get_team_robot_a_belief() / 255.0;
//    team_robot_a.push_back(team_robot_a_msg);
//
//    //team_robot_b
//    humanoid_league_msgs::ObstacleRelative team_robot_b_msg;
//    team_robot_b_msg.pose.pose.pose.position.x = rob_data.get_team_robot_b_x() / 1000.0;
//    team_robot_b_msg.pose.pose.pose.position.y = rob_data.get_team_robot_b_y() / 1000.0;
//    //team_robot_b_msg.confidence = rob_data.get_team_robot_b_belief() / 255.0;
//    team_robot_b.push_back(team_robot_b_msg);
//
//    //team_robot_c
//    humanoid_league_msgs::ObstacleRelative team_robot_c_msg;
//    team_robot_c_msg.pose.pose.pose.position.x = rob_data.get_team_robot_c_x() / 1000.0;
//    team_robot_c_msg.pose.pose.pose.position.y = rob_data.get_team_robot_c_y() / 1000.0;
//    //team_robot_c_msg.confidence = rob_data.get_team_robot_c_belief() / 255.0;
//    team_robot_c.push_back(team_robot_c_msg);
//
//    avg_walking_speeds.push_back(rob_data.get_avg_walking_speed() / 1000.0);
//    time_to_position_at_balls.push_back(rob_data.get_time_to_ball());
//    max_kicking_distances.push_back(rob_data.get_max_kicking_distance() / 1000.0);
//
//    offensive_side.push_back(rob_data.get_offensive_side());
//  }
//
  // build message
  humanoid_league_msgs::TeamData message;
  message.header.stamp.sec = google::protobuf::util::TimeUtil::TimestampToSeconds(received_msg->timestamp());
  message.header.stamp.nsec =
      google::protobuf::util::TimeUtil::TimestampToNanoseconds(received_msg->timestamp()) % int(pow(10, 9));
//
//    //Due to the message refactoring, the team data msg only includes information of one robot. This counteracts the
//    // Mitecom concept. Therefore, no received information will be published.
//    /**
//    message.robot_ids = ids;
//
//    message.role = roles;
//    message.action = actions;
//    message.state = states;
//
//    message.robot_position = own_position;
//    //own_position.confidence = own_position_beliefs;   unnecessary because of TeamData.msg
//
//    message.ball_relative = ball_relative;
//
//    //message.oppgoal_relative = oppgoal_relative;
//
//    obstacles.push_back(opponent_robot_a);
//    obstacles.push_back(opponent_robot_b);
//    obstacles.push_back(opponent_robot_c);
//    obstacles.push_back(opponent_robot_d);
//    obstacles.push_back(team_robot_a);
//    obstacles.push_back(team_robot_b);
//    obstacles.push_back(team_robot_c);
//    message.obstacles = obstacles
//
//    message.avg_walking_speed = avg_walking_speeds;
//    message.time_to_position_at_ball = time_to_position_at_balls;
//    message.max_kicking_distance = max_kicking_distances;
//
//    message.strategy = offensive_side;
//    **/
//  publisher_.publish(message);
}

void TeamCommunication::strategyCallback(humanoid_league_msgs::Strategy msg) {
  role_ = msg.role;
  action_ = msg.action;
  offensive_side_ = msg.offensive_side;
  offensive_side_set_ = true;
}

void TeamCommunication::robotStateCallback(humanoid_league_msgs::RobotControlState msg) {
  state_ = msg.state;
  // states in which the robot is penalized by the game controller
  if (state_ == humanoid_league_msgs::RobotControlState::PENALTY ||
      state_ == humanoid_league_msgs::RobotControlState::PENALTY_ANIMATION ||
      state_ == humanoid_league_msgs::RobotControlState::PICKED_UP) {
    state_ = PENALISED;
  }
    // state in which the robot is not able to play
  else if (state_ == humanoid_league_msgs::RobotControlState::STARTUP ||
      state_ == humanoid_league_msgs::RobotControlState::SHUTDOWN ||
      state_ == humanoid_league_msgs::RobotControlState::RECORD ||
      state_ == humanoid_league_msgs::RobotControlState::HCM_OFF ||
      state_ == humanoid_league_msgs::RobotControlState::HARDWARE_PROBLEM) {
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
  time_to_position_at_ball_set_ = true;

  // set time to decide if the information is up to date when broadcasting it
  ball_exists_ = msg.header.stamp.toSec();
}

void TeamCommunication::goalCallback(const humanoid_league_msgs::PoseWithCertaintyArray &msg) {
  /* todo von python nach c++ (es gibt keine positions in der msg)
   * todo improve computation of position
   * if msg.positions is None or msg.positions == []:
   * return
   * post_a = msg.positions[0]
   * if len(msg.positions) > 1:
   * post_b = msg.positions[1]
   * self.oppgoal_relative_x_ = (post_a.x - post_b.x) / 2 + post_a.x
   * self.oppgoal_relative_y_ = (post_a.y - post_b.y) / 2 + post_a.y
   * else:
   * self.oppgoal_relative_x_ = post_a.x
   * self.oppgoal_relative_y_ = post_a.y
   * self.oppgoal_belief_ = msg.confidence
   */

}

void TeamCommunication::obstaclesCallback(const humanoid_league_msgs::ObstacleRelativeArray& msg){
  // clear team_robots_ and obstacle:robots because of new data from vision
  team_robots_.clear();
  opponent_robots_.clear();

  // team color
  uint8_t opponent_color;
  if (team_color_ == humanoid_league_msgs::ObstacleRelative::ROBOT_MAGENTA) {
    opponent_color = humanoid_league_msgs::ObstacleRelative::ROBOT_CYAN;
  }
  else if (team_color_ == humanoid_league_msgs::ObstacleRelative::ROBOT_CYAN) {
    opponent_color = humanoid_league_msgs::ObstacleRelative::ROBOT_MAGENTA;
  }
  else {
    ROS_INFO_STREAM("Could not set the input \""
                        << team_color_
                        << "\" as team color. the value has to correspond with either ROBOT_MAGENTA or ROBOT_CYAN set in the ObstacleRelative message");
    return;
  }
  uint64_t x;
  uint64_t y;
  uint64_t belief = 0;

  for (auto const& obstacle : msg.obstacles){
    //only take obstacles that are team mates or opponents
    if( obstacle.type == team_color_)
    {
      x = static_cast<uint64_t>(obstacle.pose.pose.pose.position.x * 1000.0);
      y = static_cast<uint64_t>(obstacle.pose.pose.pose.position.y * 1000.0);
      //belief = static_cast<uint64_t>(obstacle.pose.confidence * 255.0); //TODO confidence
      team_robots_.push_back({x, y, belief});
    }
    else if (obstacle.type == opponent_color){
      x = static_cast<uint64_t>(obstacle.pose.pose.pose.position.x * 1000.0);
      y = static_cast<uint64_t>(obstacle.pose.pose.pose.position.y * 1000.0);
      //belief = static_cast<uint64_t>(obstacle.pose.confidence * 255.0);
      opponent_robots_.push_back({x, y, belief});
    }
  }
  obstacles_exists_ = ros::Time::now().sec;
}

int main(int argc, char **argv) {
  ROS_INFO("Starting Team Communication");
  ros::init(argc, argv, "humanoid_league_team_communication");
  // init node
  TeamCommunication node = TeamCommunication();
  // run the node
  node.run();
  ros::spin();
}
