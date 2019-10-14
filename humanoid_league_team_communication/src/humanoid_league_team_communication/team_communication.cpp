#include "humanoid_league_team_communication/team_communication.h"

TeamCommunication::TeamCommunication() : nh_() {
    //--- Params ---
    int port;
    int team;
    int player;
    nh_.getParam("team_communication/port", port);
    nh_.getParam("team_id", team);
    nh_.getParam("bot_id", player);
    //publishing rate in Hz
    nh_.getParam("team_communication/rate", frequency_);
    nh_.getParam("team_communication/avg_walking_speed", avg_walking_speed_);
    nh_.getParam("team_communication/max_kicking_distance", max_kicking_distance_);
    int teamcolor;
    nh_.getParam("team_communication/team_color", teamcolor);
  team_color_ = teamcolor;
    nh_.getParam("team_communication/world_model", world_model_);
    nh_.getParam("team_communication/lifetime", lifetime_);
    nh_.getParam("team_communication/belief_threshold", belief_threshold_);

    nh_.getParam("team_communication/team_data", teamdata_topic_);
    nh_.getParam("team_communication/strategy", strategy_topic_);
    nh_.getParam("team_communication/robot_state", robot_state_topic_);
    nh_.getParam("team_communication/goal", goal_topic_);
    nh_.getParam("team_communication/world_model_node", world_model_topic_);
    nh_.getParam("team_communication/position", position_topic_);
    nh_.getParam("team_communication/ball", ball_topic_);
    nh_.getParam("team_communication/obstacles", obstacles_topic_);

    //init mitecom
    mitecom_.set_team_id(team);
    mitecom_.open_socket(port);
    mitecom_.set_robot_id(player);

    // --- Initialize Topics ---
    publisher_ = nh_.advertise<humanoid_league_msgs::TeamData>(teamdata_topic_, 10);

  sub_role_ = nh_.subscribe(strategy_topic_, 1, &TeamCommunication::strategyCallback, this,
                            ros::TransportHints().tcpNoDelay());
  sub_robot_state_ = nh_.subscribe(robot_state_topic_, 1, &TeamCommunication::robotStateCallback,
                                   this, ros::TransportHints().tcpNoDelay());
  sub_goal_ = nh_.subscribe(goal_topic_, 1, &TeamCommunication::goalCallback, this,
                            ros::TransportHints().tcpNoDelay());

    if(world_model_){
      sub_world_ = nh_.subscribe(world_model_topic_, 1, &TeamCommunication::worldCallback, this,
                                 ros::TransportHints().tcpNoDelay());
    }
    else{
      sub_position_ = nh_.subscribe(position_topic_, 1, &TeamCommunication::positionCallback, this,
                                    ros::TransportHints().tcpNoDelay());
      sub_ball_ = nh_.subscribe(ball_topic_, 1, &TeamCommunication::ballCallback, this,
                                ros::TransportHints().tcpNoDelay());
      sub_obstacles_ = nh_.subscribe(obstacles_topic_, 1, &TeamCommunication::obstaclesCallback,
                                     this, ros::TransportHints().tcpNoDelay());
    }
}

void TeamCommunication::run() {
    // TODO: std::thread nutzen?
    pthread_t thread;
  pthread_create(&thread, nullptr, TeamCommunication::startRecvThread, this);
  timer_ = nh_.createTimer(ros::Duration(1.0f / frequency_), &TeamCommunication::sendThread, this);
    /*ros::Rate rate(frequency_);
    while(ros::ok())
    {
        sendThread();
        // let the main thread wait
        rate.sleep();
    }*/
}

void *TeamCommunication::startRecvThread(void * context) {
  ((TeamCommunication *) context)->recvThread();
}

void TeamCommunication::recvThread() {
    ros::Rate thread_rate(TeamCommunication::frequency_);
    while(ros::ok())
    {
        mitecom_.recieve_data();
        const MiTeCom::TeamRobotData* orig_data = mitecom_.get_data();
      publishData(*orig_data);
        thread_rate.sleep();
    }
}

void TeamCommunication::sendThread(const ros::TimerEvent&) {
    if (state_ != STATE_PENALIZED) {
        //state
        mitecom_.set_state(state_);
        mitecom_.set_action(action_);
        mitecom_.set_role(role_);

        mitecom_.set_max_kicking_distance(max_kicking_distance_);
        mitecom_.set_get_avg_walking_speed(avg_walking_speed_);

        //position
        if (position_belief_ > belief_threshold_ && ros::Time::now().sec - position_exists_ < lifetime_) {
            mitecom_.set_pos(position_x_, position_y_, position_orientation_, position_belief_);
        }
        //ball
        if (ball_belief_ > belief_threshold_ && ros::Time::now().sec - ball_exists_ < lifetime_) {
            mitecom_.set_relative_ball(ball_relative_x_, ball_relative_y_, ball_belief_);
        }
        else{
            // workaround to be able to identify in TeamData Msg which robot has sent the useful data
            mitecom_.set_relative_ball(1000.0, 1000.0, 0.0);
        }
        /*//opponent goal
        if (oppgoal_belief_ > 0) {
            mitecom_.set_opp_goal_relative(oppgoal_relative_x_, oppgoal_relative_y_, oppgoal_belief_);
        }*/
        if(ros::Time::now().sec - obstacles_exists_ < lifetime_) {
            //opponent robots
            if (!opponent_robots_.empty()) {
                mitecom_.set_opponent_robot_a(opponent_robots_[0][0], opponent_robots_[0][1], opponent_robots_[0][2]);
            }
            else{
                // workaround to be able to identify in TeamData Msg which robot has sent the useful data
                mitecom_.set_opponent_robot_a(1000.0, 1000.0, 0.0);
            }
            if (opponent_robots_.size() > 1) {
                mitecom_.set_opponent_robot_b(opponent_robots_[1][0], opponent_robots_[1][1], opponent_robots_[1][2]);
            }
            else{
                mitecom_.set_opponent_robot_b(1000.0, 1000.0, 0.0);
            }
            if (opponent_robots_.size() > 2) {
                mitecom_.set_opponent_robot_c(opponent_robots_[2][0], opponent_robots_[2][1], opponent_robots_[2][2]);
            }
            else{
                mitecom_.set_opponent_robot_c(1000.0, 1000.0, 0.0);
            }
            if (opponent_robots_.size() > 3) {
                mitecom_.set_opponent_robot_d(opponent_robots_[3][0], opponent_robots_[3][1], opponent_robots_[3][2]);
            }
            else{
                mitecom_.set_opponent_robot_d(1000.0, 1000.0, 0.0);
            }

            //team robots
            if (!team_robots_.empty()) {
                mitecom_.set_team_robot_a(team_robots_[0][0], team_robots_[0][1], team_robots_[0][2]);
            }
            else{
                mitecom_.set_team_robot_a(1000.0, 1000.0, 0.0);
            }
            if (team_robots_.size() > 1) {
                mitecom_.set_team_robot_b(team_robots_[1][0], team_robots_[1][1], team_robots_[1][2]);
            }
            else{
                mitecom_.set_team_robot_b(1000.0, 1000.0, 0.0);
            }
            if (team_robots_.size() > 2) {
                mitecom_.set_team_robot_c(team_robots_[2][0], team_robots_[2][1], team_robots_[2][2]);
            }
            else{
                mitecom_.set_team_robot_c(1000.0, 1000.0, 0.0);
            }
        }
        else{
            // workaround to be able to identify in TeamData Msg which robot has sent the useful data
            mitecom_.set_opponent_robot_a(1000.0, 1000.0, 0.0);
            mitecom_.set_opponent_robot_b(1000.0, 1000.0, 0.0);
            mitecom_.set_opponent_robot_c(1000.0, 1000.0, 0.0);
            mitecom_.set_opponent_robot_d(1000.0, 1000.0, 0.0);
            mitecom_.set_team_robot_a(1000.0, 1000.0, 0.0);
            mitecom_.set_team_robot_b(1000.0, 1000.0, 0.0);
            mitecom_.set_team_robot_c(1000.0, 1000.0, 0.0);

        }

        //time to ball
        if(time_to_position_at_ball_set_ && ros::Time::now().sec - ball_exists_ < lifetime_) {
            mitecom_.set_time_to_ball(time_to_position_at_ball_);
        }
        // strategy
        if(offensive_side_set_) {
            mitecom_.set_offensive_side(offensive_side_);
        }
        mitecom_.send_data();
    }
}

void TeamCommunication::publishData(const MiTeCom::TeamRobotData& team_data){
    std::vector<uint8_t> ids;
    std::vector<uint8_t> roles;
    std::vector<uint8_t> actions;
    std::vector<uint8_t> states;
    std::vector<geometry_msgs::Pose2D> own_position;
    //std::vector<uint8_t> own_position_beliefs;   unnecessary because of TeamData.msg
    std::vector<humanoid_league_msgs::Position2D> ball_relative;
    //std::vector<humanoid_league_msgs::Position2D> oppgoal_relative;
    std::vector<humanoid_league_msgs::Position2D> opponent_robot_a;
    std::vector<humanoid_league_msgs::Position2D> opponent_robot_b;
    std::vector<humanoid_league_msgs::Position2D> opponent_robot_c;
    std::vector<humanoid_league_msgs::Position2D> opponent_robot_d;
    std::vector<humanoid_league_msgs::Position2D> team_robot_a;
    std::vector<humanoid_league_msgs::Position2D> team_robot_b;
    std::vector<humanoid_league_msgs::Position2D> team_robot_c;
    std::vector<float> avg_walking_speeds;
    std::vector<float> time_to_position_at_balls;
    std::vector<float> max_kicking_distances;
    std::vector<uint8_t> offensive_side;

    //iterate through all robots from which we received data
    for (auto const& x : team_data) {
        MiTeCom::TeamMateData rob_data;
        rob_data = *x.second;

        ids.push_back(rob_data.get_id());
        roles.push_back(rob_data.get_role());
        actions.push_back(rob_data.get_action());
        states.push_back(rob_data.get_state());

        //own position
        //TODO position confidence -> msg definitions
        geometry_msgs::Pose2D pos_msg;
        pos_msg.x = rob_data.get_absolute_x() / 1000.0;
        pos_msg.y = rob_data.get_absolute_y() / 1000.0;
        pos_msg.theta = rob_data.get_absolute_orientation() / 1000.0;
        own_position.push_back(pos_msg);
        //own_position_beliefs.push_back(rob_data.get_absolute_belief() / 255.0);   unnecessary because of TeamData.msg

        //ball
        humanoid_league_msgs::Position2D ball_msg;
        ball_msg.pose.x = rob_data.get_relative_ball_x() / 1000.0;
        ball_msg.pose.y = rob_data.get_relative_ball_y() / 1000.0;
        ball_msg.confidence = rob_data.get_ball_belief() / 255.0;
        ball_relative.push_back(ball_msg);

        /*//oppgoal
        humanoid_league_msgs::Position2D oppgoal_msg;
        oppgoal_msg.pose.x = rob_data.get_oppgoal_relative_x() / 1000.0;
        oppgoal_msg.pose.y = rob_data.get_oppgoal_relative_y() / 1000.0;
        oppgoal_msg.confidence = rob_data.get_oppgoal_belief() / 255.0;
        oppgoal_relative.push_back(oppgoal_msg);*/

        //opponent_robot_a
        humanoid_league_msgs::Position2D opponent_robot_a_msg;
        opponent_robot_a_msg.pose.x = rob_data.get_opponent_robot_a_x() / 1000.0;
        opponent_robot_a_msg.pose.y = rob_data.get_opponent_robot_a_y() / 1000.0;
        opponent_robot_a_msg.confidence = rob_data.get_opponent_robot_a_belief() / 255.0;
        opponent_robot_a.push_back(opponent_robot_a_msg);

        //opponent_robot_b
        humanoid_league_msgs::Position2D opponent_robot_b_msg;
        opponent_robot_b_msg.pose.x = rob_data.get_opponent_robot_b_x() / 1000.0;
        opponent_robot_b_msg.pose.y = rob_data.get_opponent_robot_b_y() / 1000.0;
        opponent_robot_b_msg.confidence = rob_data.get_opponent_robot_b_belief() / 255.0;
        opponent_robot_b.push_back(opponent_robot_b_msg);

        //opponent_robot_c
        humanoid_league_msgs::Position2D opponent_robot_c_msg;
        opponent_robot_c_msg.pose.x = rob_data.get_opponent_robot_c_x() / 1000.0;
        opponent_robot_c_msg.pose.y = rob_data.get_opponent_robot_c_y() / 1000.0;
        opponent_robot_c_msg.confidence = rob_data.get_opponent_robot_c_belief() / 255.0;
        opponent_robot_c.push_back(opponent_robot_c_msg);

        //opponent_robot_d
        humanoid_league_msgs::Position2D opponent_robot_d_msg;
        opponent_robot_d_msg.pose.x = rob_data.get_opponent_robot_d_x() / 1000.0;
        opponent_robot_d_msg.pose.y = rob_data.get_opponent_robot_d_y() / 1000.0;
        opponent_robot_d_msg.confidence = rob_data.get_opponent_robot_d_belief() / 255.0;
        opponent_robot_d.push_back(opponent_robot_d_msg);

        //team_robot_a
        humanoid_league_msgs::Position2D team_robot_a_msg;
        team_robot_a_msg.pose.x = rob_data.get_team_robot_a_x() / 1000.0;
        team_robot_a_msg.pose.y = rob_data.get_team_robot_a_y() / 1000.0;
        team_robot_a_msg.confidence = rob_data.get_team_robot_a_belief() / 255.0;
        team_robot_a.push_back(team_robot_a_msg);

        //team_robot_b
        humanoid_league_msgs::Position2D team_robot_b_msg;
        team_robot_b_msg.pose.x = rob_data.get_team_robot_b_x() / 1000.0;
        team_robot_b_msg.pose.y = rob_data.get_team_robot_b_y() / 1000.0;
        team_robot_b_msg.confidence = rob_data.get_team_robot_b_belief() / 255.0;
        team_robot_b.push_back(team_robot_b_msg);

        //team_robot_c
        humanoid_league_msgs::Position2D team_robot_c_msg;
        team_robot_c_msg.pose.x = rob_data.get_team_robot_c_x() / 1000.0;
        team_robot_c_msg.pose.y = rob_data.get_team_robot_c_y() / 1000.0;
        team_robot_c_msg.confidence = rob_data.get_team_robot_c_belief() / 255.0;
        team_robot_c.push_back(team_robot_c_msg);

        avg_walking_speeds.push_back(rob_data.get_avg_walking_speed() / 1000.0);
        time_to_position_at_balls.push_back(rob_data.get_time_to_ball());
        max_kicking_distances.push_back(rob_data.get_max_kicking_distance() / 1000.0);

        offensive_side.push_back(rob_data.get_offensive_side());
    }

    // build message
    humanoid_league_msgs::TeamData message;
    message.header.stamp = ros::Time::now();

    message.robot_ids = ids;

    message.role = roles;
    message.action = actions;
    message.state = states;

    message.robot_positions = own_position;
    //own_position.confidence = own_position_beliefs;   unnecessary because of TeamData.msg

    message.ball_relative = ball_relative;

    //message.oppgoal_relative = oppgoal_relative;

    message.opponent_robot_a = opponent_robot_a;
    message.opponent_robot_b = opponent_robot_b;
    message.opponent_robot_c = opponent_robot_c;
    message.opponent_robot_d = opponent_robot_d;

    message.team_robot_a = team_robot_a;
    message.team_robot_b = team_robot_b;
    message.team_robot_c = team_robot_c;

    message.avg_walking_speed = avg_walking_speeds;
    message.time_to_position_at_ball = time_to_position_at_balls;
    message.max_kicking_distance = max_kicking_distances;

    message.offensive_side = offensive_side;

    publisher_.publish(message);
}

void TeamCommunication::strategyCallback(humanoid_league_msgs::Strategy msg) {
  role_ = msg.role;
  action_ = msg.action;
  offensive_side_ = msg.offensive_side;
  offensive_side_set_ = true;
}

void TeamCommunication::robotStateCallback(humanoid_league_msgs::RobotControlState msg) {
  state_ = msg.state;
    if (state_ == humanoid_league_msgs::RobotControlState::PENALTY ||
        state_ == humanoid_league_msgs::RobotControlState::PENALTY_ANIMANTION ||
        state_ == humanoid_league_msgs::RobotControlState::PICKED_UP){
      state_ = STATE_PENALIZED;
    }
    else if (state_ == humanoid_league_msgs::RobotControlState::STARTUP ||
             state_ == humanoid_league_msgs::RobotControlState::SHUTDOWN ||
             state_ == humanoid_league_msgs::RobotControlState::RECORD) {
      state_ = STATE_INACTIVE;
    }
    else {
      state_ = STATE_ACTIVE;
    }
}

void TeamCommunication::positionCallback(const humanoid_league_msgs::Position2D& msg) {
    //conversion from m (ROS message) to mm (self.mitecom)
    position_x_ = static_cast<uint64_t>(msg.pose.x * 1000.0);
  position_y_ = static_cast<uint64_t>(msg.pose.y * 1000.0);
  position_orientation_ = static_cast<uint64_t>(msg.pose.theta * 1000.0);
    //the scale is different in mitecom_, so we have to transfer from 0...1 to 0...255
    position_belief_ = static_cast<uint64_t>(msg.confidence * 255.0);
  position_exists_ = msg.header.stamp.sec;
}

void TeamCommunication::ballCallback(const humanoid_league_msgs::BallRelative& msg){
    //conversion from m (ROS message) to mm (self.mitecom)
    ball_relative_x_ = static_cast<uint64_t>(msg.ball_relative.x * 1000.0);
  ball_relative_y_ = static_cast<uint64_t>(msg.ball_relative.y * 1000.0);
    //the scale is different in mitecom_, so we have to transfer from 0...1 to 0...255
    ball_belief_ = static_cast<uint64_t>(msg.confidence * 255.0);
    //use pythagoras to compute time to ball
    time_to_position_at_ball_ = static_cast<uint64_t>((sqrt((pow(msg.ball_relative.x, 2.0) + pow(msg.ball_relative.y, 2.0))) * 1000.0) / avg_walking_speed_);
  time_to_position_at_ball_set_ = true;
  ball_exists_ = msg.header.stamp.sec;
}

void TeamCommunication::goalCallback(const humanoid_league_msgs::GoalRelative& msg) {
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

void TeamCommunication::obstaclesCallback(const humanoid_league_msgs::ObstaclesRelative& msg){
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
    uint64_t belief;

    for (auto const& obstacle : msg.obstacles){
        //only take obstacles that are team mates or opponents
        if( obstacle.color == team_color_)
        {
            x = static_cast<uint64_t>(obstacle.position.x * 1000.0);
            y = static_cast<uint64_t>(obstacle.position.y * 1000.0);
            belief = static_cast<uint64_t>(obstacle.confidence * 255.0);
            team_robots_.push_back({x, y, belief});
        }
        else if (obstacle.color == opponent_color){
            x = static_cast<uint64_t>(obstacle.position.x * 1000.0);
            y = static_cast<uint64_t>(obstacle.position.y * 1000.0);
            belief = static_cast<uint64_t>(obstacle.confidence * 255.0);
            opponent_robots_.push_back({x, y, belief});
        }
    }
  obstacles_exists_ = ros::Time::now().sec;
}

void TeamCommunication::worldCallback(const humanoid_league_msgs::Model& msg){
    //ball
  ballCallback(msg.ball);

    //position
    //conversion from m (ROS message) to mm (self.mitecom)
    position_x_ = static_cast<uint64_t>(msg.position.pose.pose.position.x * 1000);
  position_y_ = static_cast<uint64_t>(msg.position.pose.pose.position.y * 1000);
    tf::Quaternion quaternion(msg.position.pose.pose.orientation.x, msg.position.pose.pose.orientation.y,
                              msg.position.pose.pose.orientation.z, msg.position.pose.pose.orientation.w);
    tf::Matrix3x3 matrix_quaternion(quaternion);
    double roll, pitch, yaw;
    matrix_quaternion.getRPY(roll, pitch, yaw);
  position_orientation_ = static_cast<uint64_t>(yaw * 1000.0);
    // convert covariance matrix to confidence, so Mitecom can send it as int
    double sum = 0;
    int elements = 0;
    for (auto const& matrixelement : msg.position.pose.covariance){
        sum += matrixelement;
        if (matrixelement > 0){
            // count number of significant elements in the matrix
            elements += 1;
        }
    }
    // TODO sinnvollerere Normalisierung
    double covariance_mean = sum/elements;
    //the scale is different in mitecom_, so we have to transfer from 0...1 to 0...255
    position_belief_ = static_cast<uint64_t>(covariance_mean * 255);
    ROS_INFO_STREAM(covariance_mean);

    //obstacles
  obstaclesCallback(msg.obstacles);
}


int main(int argc, char **argv){
    ROS_INFO("Starting Team Communication");
    ros::init(argc, argv, "humanoid_league_team_communication");
    // init node
    TeamCommunication node = TeamCommunication();
    // run the node
    node.run();
    ros::spin();
}

