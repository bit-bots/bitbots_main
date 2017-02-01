from libcpp.map cimport map


cdef import from "mitecom.hpp" namespace "MiTeCom":
    cdef cppclass _TeamMateData "MiTeCom::TeamMateData":
        int get_id()
        int get_role()
        int get_action()
        int get_state()
        
        int get_absolute_x()
        int get_absolute_y()
        int get_absolute_orientation()
        int get_absolute_belief()
        int get_relative_ball_x()
        int get_relative_ball_y()
        int get_ball_belief()
        int get_oppgoal_relative_x()
        int get_oppgoal_relative_y()
        int get_oppgoal_belief()
        int get_opponent_robot_a_x()
        int get_opponent_robot_a_y()
        int get_opponent_robot_a_belief()
        int get_opponent_robot_b_x()
        int get_opponent_robot_b_y()
        int get_opponent_robot_b_belief()
        int get_opponent_robot_c_x()
        int get_opponent_robot_c_y()
        int get_opponent_robot_c_belief()
        int get_opponent_robot_d_x()
        int get_opponent_robot_d_y()
        int get_opponent_robot_d_belief()
        int get_team_robot_a_x()
        int get_team_robot_a_y()
        int get_team_robot_a_belief()
        int get_team_robot_b_x()
        int get_team_robot_b_y()
        int get_team_robot_b_belief()
        int get_team_robot_c_x()
        int get_team_robot_c_y()
        int get_team_robot_c_belief()

        int get_avg_walking_speed()
        int get_time_to_ball()
        int get_max_kicking_distance()

        int get_offencive_side()

    cdef cppclass mitecom:
        void mietcom()
        #void ~mietcom()
        void open_socket(int port)
        void send_data()
        void recieve_data()

        void set_robot_id(int robot_id)
        void set_team_id(int team_id)

        void set_role(int role)
        void set_action(int action)
        void set_state(int state)

        void set_pos(int x, int y, int orientation, int belief)
        void set_relative_ball(int x, int y, int belief)
        void set_opp_goal_relative(int x, int y, int belief)
        void set_opponent_robot_a(int x, int y, int belief)
        void set_opponent_robot_b(int x, int y, int belief)
        void set_opponent_robot_c(int x, int y, int belief)
        void set_opponent_robot_d(int x, int y, int belief)
        void set_team_robot_a(int x, int y, int belief)
        void set_team_robot_b(int x, int y, int belief)
        void set_team_robot_c(int x, int y, int belief)

        void set_get_avg_walking_speed(int speed)
        void set_time_to_ball(int sec)
        void set_max_kicking_distance(int distance)

        void set_offencive_side(int side)

        const map[int,_TeamMateData*] *get_data()

