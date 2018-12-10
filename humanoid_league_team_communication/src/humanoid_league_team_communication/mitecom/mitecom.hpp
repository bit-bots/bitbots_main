
#include "mitecom-data.h"
#include "mitecom-network.h"
#include "mitecom-handler.h"
#include <ros/console.h>

namespace MiTeCom {

class TeamMateData {
    public:
        void setData(MixedTeamMate mate);
        int get_id(void);
        int get_role(void);
        int get_action(void);
        int get_state(void);

        int get_absolute_x(void);
        int get_absolute_y(void);
        int get_absolute_orientation(void);
        int get_absolute_belief(void);
        int get_relative_ball_x(void);
        int get_relative_ball_y(void);
        int get_ball_belief(void);
        int get_oppgoal_relative_x(void);
        int get_oppgoal_relative_y(void);
        int get_oppgoal_belief(void);
        int get_opponent_robot_a_x(void);
        int get_opponent_robot_a_y(void);
        int get_opponent_robot_a_belief(void);
        int get_opponent_robot_b_x(void);
        int get_opponent_robot_b_y(void);
        int get_opponent_robot_b_belief(void);
        int get_opponent_robot_c_x(void);
        int get_opponent_robot_c_y(void);
        int get_opponent_robot_c_belief(void);
        int get_opponent_robot_d_x(void);
        int get_opponent_robot_d_y(void);
        int get_opponent_robot_d_belief(void);
        int get_team_robot_a_x(void);
        int get_team_robot_a_y(void);
        int get_team_robot_a_belief(void);
        int get_team_robot_b_x(void);
        int get_team_robot_b_y(void);
        int get_team_robot_b_belief(void);
        int get_team_robot_c_x(void);
        int get_team_robot_c_y(void);
        int get_team_robot_c_belief(void);

        int get_avg_walking_speed(void);
        int get_time_to_ball(void);
        int get_max_kicking_distance(void);

        int get_offensive_side(void);

        void set_lastUpdate(uint64_t);
        uint64_t get_lastUpdate(void);

    private:
        MixedTeamMate data;
        uint64_t m_lastUpdate;
};


typedef std::map<int, TeamMateData*> TeamRobotData;

class mitecom {
    public:
        mitecom(void);
        ~mitecom(void);
        void open_socket(int port);
        void send_data(void);
        void recieve_data(void);

        void set_robot_id(int ID);
        void set_team_id(int id);

        void set_role(int role);
        void set_action(int action);
        void set_state(int state);

        void set_pos(int x, int y, int orientation, int belief);
        void set_relative_ball(int x, int y, int belief);
        void set_opp_goal_relative(int x, int y, int belief);
        void set_opponent_robot_a(int x, int y, int belief);
        void set_opponent_robot_b(int x, int y, int belief);
        void set_opponent_robot_c(int x, int y, int belief);
        void set_opponent_robot_d(int x, int y, int belief);
        void set_team_robot_a(int x, int y, int belief);
        void set_team_robot_b(int x, int y, int belief);
        void set_team_robot_c(int x, int y, int belief);

        void set_get_avg_walking_speed(int speed);
        void set_time_to_ball(int sec);
        void set_max_kicking_distance(int distance);

        void set_offensive_side(int side);

        const TeamRobotData *get_data(void);

    private:
        MixedTeamMate* ownData;
        MixedTeamMates teamMates;
        int m_robotID;
        int m_sock;
        int m_teamID;
        int m_port;
        TeamRobotData teamRobotData;
};

} //namespace MiTeCom
