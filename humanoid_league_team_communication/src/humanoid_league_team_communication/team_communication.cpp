#include "humanoid_league_team_communication/team_communication.hpp"

TeamCommunication::TeamCommunication() {
    ros::NodeHandle _nh;
    //--- Params ---
    int port;
    int team;
    int player;
    _nh.getParam("team_communication/port", port);
    _nh.getParam("team_id", team);
    _nh.getParam("bot_id", player);

    //publishing rate in Hz
    _nh.getParam("team_communication/rate", frequency);
    _nh.getParam("team_communication/avg_walking_speed", avg_walking_speed);
    _nh.getParam("team_communication/max_kicking_distance", max_kicking_distance);
    int teamcolor;
    _nh.getParam("team_communication/team_color", teamcolor);
    team_color = teamcolor;

    //init mitecom
    _mitecom.set_team_id(team);
    _mitecom.open_socket(port);
    _mitecom.set_robot_id(player);

    // --- Initialize Topics ---
    publisher = _nh.advertise<humanoid_league_msgs::TeamData>("/team_data", 10);

    ros::Subscriber sub_role = _nh.subscribe("role", 1, &TeamCommunication::strategy_callback, this,
            ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_position = _nh.subscribe("position", 1, &TeamCommunication::position_callback, this,
            ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_motion_state = _nh.subscribe("motion_state", 1, &TeamCommunication::motion_state_callback,
            this, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_ball = _nh.subscribe("ball_relative", 1, &TeamCommunication::ball_callback, this,
            ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_goal = _nh.subscribe("goal_relative", 1, &TeamCommunication::goal_callback, this,
            ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_obstacles = _nh.subscribe("obstacle_relative", 1, &TeamCommunication::obstacles_callback,
            this, ros::TransportHints().tcpNoDelay());
}

void TeamCommunication::run() {
    // TODO: std::thread nutzen?
    pthread_t thread;
    pthread_create (&thread, NULL, TeamCommunication::start_recv_thread, this);
    ros::Rate rate(frequency);
    while(ros::ok())
    {
        send_thread();
        // let the main thread wait
        rate.sleep();
    }
}

void *TeamCommunication::start_recv_thread(void * context) {
    ((TeamCommunication *)context)->recv_thread();
}

void TeamCommunication::recv_thread() {
    ros::Rate thread_rate(TeamCommunication::frequency);
    while(ros::ok())
    {
        _mitecom.recieve_data();
        const MiTeCom::TeamRobotData* orig_data = _mitecom.get_data();
        publish_data(*orig_data);
        thread_rate.sleep();
    }
}

void TeamCommunication::send_thread() {
    //state
    _mitecom.set_state(state);
    _mitecom.set_action(action);
    _mitecom.set_role(role);
    if (state != STATE_PENALIZED) {
        //position
        if (position_belief > 0) {
            _mitecom.set_pos(position_x, position_y, position_orientation, position_belief);
        }
        //ball
        if (ball_belief > 0) {
            _mitecom.set_relative_ball(ball_relative_x, ball_relative_y, ball_belief);
        }
        //opponent goal
        if (oppgoal_belief > 0) {
            _mitecom.set_opp_goal_relative(oppgoal_relative_x, oppgoal_relative_y, oppgoal_belief);
        }
        //opponent robots
        if (opponent_robots.size() > 0) {
            _mitecom.set_opponent_robot_a(opponent_robots[0][0], opponent_robots[0][1], opponent_robots[0][2]);
        }
        if (opponent_robots.size() > 1) {
            _mitecom.set_opponent_robot_b(opponent_robots[1][0], opponent_robots[1][1], opponent_robots[1][2]);
        }
        if (opponent_robots.size() > 2) {
            _mitecom.set_opponent_robot_c(opponent_robots[2][0], opponent_robots[2][1], opponent_robots[2][2]);
        }
        if (opponent_robots.size() > 3) {
            _mitecom.set_opponent_robot_d(opponent_robots[3][0], opponent_robots[3][1], opponent_robots[3][2]);
        }

        //team robots
        if (team_robots.size() > 0) {
            _mitecom.set_team_robot_a(team_robots[0][0], team_robots[0][1], team_robots[0][1]);
        }
        if (team_robots.size() > 1) {
            _mitecom.set_team_robot_b(team_robots[1][0], team_robots[1][1], team_robots[1][2]);
        }
        if (team_robots.size() > 2) {
            _mitecom.set_team_robot_c(team_robots[2][0], team_robots[2][1], team_robots[2][2]);
        }

        //time to ball
        if(time_to_position_at_ball_set) {
            _mitecom.set_time_to_ball(time_to_position_at_ball);
        }
        // strategy
        if(offensive_side_set) {
            _mitecom.set_offensive_side(offensive_side);
        }
    }
    _mitecom.send_data();
}

void TeamCommunication::publish_data(MiTeCom::TeamRobotData team_data){
    std::vector<uint8_t> ids;
    std::vector<uint8_t> roles;
    std::vector<uint8_t> actions;
    std::vector<uint8_t> states;
    std::vector<geometry_msgs::Pose2D> own_position;
    //std::vector<uint8_t> own_position_beliefs;   unnecessary because of TeamData.msg
    std::vector<humanoid_league_msgs::Position2D> ball_relative;
    std::vector<humanoid_league_msgs::Position2D> oppgoal_relative;
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
        geometry_msgs::Pose2D pos_msg;
        pos_msg.x = rob_data.get_absolute_x() / 1000;
        pos_msg.y = rob_data.get_absolute_y() / 1000;
        pos_msg.theta = rob_data.get_absolute_orientation();
        own_position.push_back(pos_msg);
        //own_position_beliefs.push_back(rob_data.get_absolute_belief() / 255);   unnecessary because of TeamData.msg

        //ball
        humanoid_league_msgs::Position2D ball_msg;
        ball_msg.pose.x = rob_data.get_relative_ball_x() / 1000;
        ball_msg.pose.y = rob_data.get_relative_ball_y() / 1000;
        ball_msg.confidence = rob_data.get_ball_belief() / 255;
        ball_relative.push_back(ball_msg);

        //oppgoal
        humanoid_league_msgs::Position2D oppgoal_msg;
        oppgoal_msg.pose.x = rob_data.get_oppgoal_relative_x() / 1000;
        oppgoal_msg.pose.y = rob_data.get_oppgoal_relative_y() / 1000;
        oppgoal_msg.confidence = rob_data.get_oppgoal_belief() / 255;
        oppgoal_relative.push_back(oppgoal_msg);

        //opponent_robot_a
        humanoid_league_msgs::Position2D opponent_robot_a_msg;
        opponent_robot_a_msg.pose.x = rob_data.get_opponent_robot_a_x() / 1000;
        opponent_robot_a_msg.pose.y = rob_data.get_opponent_robot_a_y() / 1000;
        opponent_robot_a_msg.confidence = rob_data.get_opponent_robot_a_belief() / 255;
        opponent_robot_a.push_back(opponent_robot_a_msg);

        //opponent_robot_b
        humanoid_league_msgs::Position2D opponent_robot_b_msg;
        opponent_robot_b_msg.pose.x = rob_data.get_opponent_robot_b_x() / 1000;
        opponent_robot_b_msg.pose.y = rob_data.get_opponent_robot_b_y() / 1000;
        opponent_robot_b_msg.confidence = rob_data.get_opponent_robot_b_belief() / 255;
        opponent_robot_b.push_back(opponent_robot_b_msg);

        //opponent_robot_c
        humanoid_league_msgs::Position2D opponent_robot_c_msg;
        opponent_robot_c_msg.pose.x = rob_data.get_opponent_robot_c_x() / 1000;
        opponent_robot_c_msg.pose.y = rob_data.get_opponent_robot_c_y() / 1000;
        opponent_robot_c_msg.confidence = rob_data.get_opponent_robot_c_belief() / 255;
        opponent_robot_c.push_back(opponent_robot_c_msg);

        //opponent_robot_d
        humanoid_league_msgs::Position2D opponent_robot_d_msg;
        opponent_robot_d_msg.pose.x = rob_data.get_opponent_robot_d_x() / 1000;
        opponent_robot_d_msg.pose.y = rob_data.get_opponent_robot_d_y() / 1000;
        opponent_robot_d_msg.confidence = rob_data.get_opponent_robot_d_belief() / 255;
        opponent_robot_d.push_back(opponent_robot_d_msg);

        //team_robot_a
        humanoid_league_msgs::Position2D team_robot_a_msg;
        team_robot_a_msg.pose.x = rob_data.get_team_robot_a_x() / 1000;
        team_robot_a_msg.pose.y = rob_data.get_team_robot_a_y() / 1000;
        team_robot_a_msg.confidence = rob_data.get_team_robot_a_belief() / 255;
        team_robot_a.push_back(team_robot_a_msg);

        //team_robot_b
        humanoid_league_msgs::Position2D team_robot_b_msg;
        team_robot_b_msg.pose.x = rob_data.get_team_robot_b_x() / 1000;
        team_robot_b_msg.pose.y = rob_data.get_team_robot_b_y() / 1000;
        team_robot_b_msg.confidence = rob_data.get_team_robot_b_belief() / 255;
        team_robot_b.push_back(team_robot_b_msg);

        //team_robot_c
        humanoid_league_msgs::Position2D team_robot_c_msg;
        team_robot_c_msg.pose.x = rob_data.get_team_robot_c_x() / 1000;
        team_robot_c_msg.pose.y = rob_data.get_team_robot_c_y() / 1000;
        team_robot_c_msg.confidence = rob_data.get_team_robot_c_belief() / 255;
        team_robot_c.push_back(team_robot_c_msg);

        avg_walking_speeds.push_back(rob_data.get_avg_walking_speed());
        time_to_position_at_balls.push_back(rob_data.get_time_to_ball());
        max_kicking_distances.push_back(rob_data.get_max_kicking_distance());

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

    message.oppgoal_relative = oppgoal_relative;

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

    publisher.publish(message);
}

void TeamCommunication::strategy_callback(const humanoid_league_msgs::Strategy msg) {
    role = msg.role;
    action = msg.action;
}

void TeamCommunication::motion_state_callback(const humanoid_league_msgs::RobotControlState msg) {
    state = msg.state;
    if (state == humanoid_league_msgs::RobotControlState::PENALTY){
        state = STATE_PENALIZED;
    }
    else if (state == humanoid_league_msgs::RobotControlState::STARTUP ||
            state == humanoid_league_msgs::RobotControlState::SHUTDOWN ||
            state == humanoid_league_msgs::RobotControlState::RECORD) {
        state = STATE_INACTIVE;
    }
    else {
        state = STATE_ACTIVE;
    }
}

void TeamCommunication::position_callback(const humanoid_league_msgs::Position2D msg) {
    //conversion from m (ROS message) to mm (self.mitecom)
    position_x = int(msg.pose.x * 1000);
    position_y = int(msg.pose.y * 1000);
    position_orientation = int(msg.pose.theta);
    //the scale is different in _mitecom, so we have to transfer from 0...1 to 0...255
    position_belief = int(msg.confidence * 255);
}

void TeamCommunication::ball_callback(const humanoid_league_msgs::BallRelative msg){
    //conversion from m (ROS message) to mm (self.mitecom)
    ball_relative_x = int(msg.ball_relative.x * 1000);
    ball_relative_y = int(msg.ball_relative.y * 1000);
    //the scale is different in _mitecom, so we have to transfer from 0...1 to 0...255
    ball_belief = int(msg.confidence * 255);
    //use pythagoras to compute time to ball
    time_to_position_at_ball = sqrt((pow(ball_relative_x, 2) + pow(ball_relative_y, 2))) / avg_walking_speed;
}

void TeamCommunication::goal_callback(const humanoid_league_msgs::GoalRelative msg) {
    //todo check if this is a good way to compute the position
    /* todo von python nach c++ (es gibt keine positions in der msg)
     * if msg.positions is None or msg.positions == []:
     * return
     * post_a = msg.positions[0]
     * if len(msg.positions) > 1:
     * post_b = msg.positions[1]
     * self.oppgoal_relative_x = (post_a.x - post_b.x) / 2 + post_a.x
     * self.oppgoal_relative_y = (post_a.y - post_b.y) / 2 + post_a.y
     * else:
     * self.oppgoal_relative_x = post_a.x
     * self.oppgoal_relative_y = post_a.y
     * self.oppgoal_belief = msg.confidence
     */

}

void TeamCommunication::obstacles_callback(const humanoid_league_msgs::ObstaclesRelative msg){
    // clear team_robots and obstacle:robots because of new data from vision
    team_robots.clear();
    opponent_robots.clear();

    // team color
    uint8_t opponent_color;
    if (team_color == humanoid_league_msgs::ObstacleRelative::ROBOT_MAGENTA) {
        opponent_color = humanoid_league_msgs::ObstacleRelative::ROBOT_CYAN;
    }
    else if (team_color == humanoid_league_msgs::ObstacleRelative::ROBOT_CYAN) {
        opponent_color = humanoid_league_msgs::ObstacleRelative::ROBOT_MAGENTA;
    }
    else {
        ROS_INFO_STREAM("Could not set the input \""
                            << team_color
                            << "\" as team color. the value has to correspond with either ROBOT_MAGENTA or ROBOT_CYAN set in the ObstacleRelative message");
        return;
    }
    uint64_t x;
    uint64_t y;
    uint64_t belief;

      for (auto const& obstacle : msg.obstacles){
          //only take obstacles that are team mates or opponents
          if( obstacle.color == team_color)
          {
              x = static_cast<uint64_t>(obstacle.position.x);
              y = static_cast<uint64_t>(obstacle.position.y);
              belief = static_cast<uint64_t>(obstacle.confidence);
              team_robots.push_back({x, y, belief});
          }
          else if (obstacle.color == opponent_color){
              x = static_cast<uint64_t>(obstacle.position.x);
              y = static_cast<uint64_t>(obstacle.position.y);
              belief = static_cast<uint64_t>(obstacle.confidence);
              opponent_robots.push_back({x, y, belief});
          }
      }
}


int main(int argc, char **argv){
    ROS_INFO("Starting Team Communication");
    ros::init(argc, argv, "humanoid_league_team_communication");
    // init node
    TeamCommunication node = TeamCommunication();
    // run the node
    node.run();
}

