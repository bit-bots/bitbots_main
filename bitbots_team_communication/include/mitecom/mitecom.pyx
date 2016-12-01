from libcpp.map cimport map
from cython.operator cimport dereference as deref

cdef class TeamMateData:
    cdef _TeamMateData data

    cdef set_team_mate_data(self, const _TeamMateData &data):
        self.data = data

    cpdef int get_id(self):
        return self.data.get_id()

    cpdef int get_role(self):
        return self.data.get_role()

    cpdef int get_action(self):
        return self.data.get_action()

    cpdef int get_state(self):
        return self.data.get_state()

    cpdef int get_relative_ball_x(self):
        return self.data.get_relative_ball_x()

    cpdef int get_relative_ball_y(self):
        return self.data.get_relative_ball_y()

    cpdef int get_ball_time(self):
        cdef int time = self.data.get_ball_time()
        if time == 0:
            # 0 ist der defaultwert wenn nichts gesetzt ist
            time = 99999999
        return time

    cpdef int get_kickoff_offence_side(self):
        return self.data.get_kickoff_offence_side()


    cpdef int get_opponent_robot_x(self):
        return self.data.get_opponent_robot_x()

    cpdef int get_opponent_robot_y(self):
        return self.data.get_opponent_robot_y()

    cpdef int get_opponent_robot_3(self):
        return self.data.get_opponent_robot_3()

    cpdef int get_opponent_robot_4(self):
        return self.data.get_opponent_robot_4()

    cpdef int get_opponent_robot_5(self):
        return self.data.get_opponent_robot_5()

    cpdef int get_opponent_robot_6(self):
        return self.data.get_opponent_robot_6()

    cpdef int get_opponent_robot_7(self):
        return self.data.get_opponent_robot_7()

    cpdef int get_opponent_robot_8(self):
        return self.data.get_opponent_robot_8()

    cpdef int get_team_mate_x(self):
        return self.data.get_team_mate_x()

    cpdef int get_team_mate_y(self):
        return self.data.get_team_mate_y()

    cpdef int get_team_mate_3(self):
        return self.data.get_team_mate_3()

    cpdef int get_team_mate_4(self):
        return self.data.get_team_mate_4()

    cpdef int get_team_mate_5(self):
        return self.data.get_team_mate_5()

    cpdef int get_team_mate_6(self):
        return self.data.get_team_mate_6()

    cpdef int get_team_mate_7(self):
        return self.data.get_team_mate_7()

    cpdef int get_team_mate_8(self):
        return self.data.get_team_mate_8()

cdef class MiteCom(object):
    cdef mitecom _mitecom

    def __init__(self, int port, int team_ID):
        self._mitecom.open_socket(port)
        self._mitecom.set_team_id(team_ID)

    cpdef set_robot_id(self, int ID):
        self._mitecom.set_robot_id(ID)

    cpdef set_pos(self, int x, int y, int orientation, int belief):
        self._mitecom.set_pos(x, y, orientation, belief)

    cpdef send_data(self):
        self._mitecom.send_data()

    cpdef recieve_data(self):
        self._mitecom.recieve_data()

    cpdef set_role(self, int role):
        self._mitecom.set_role(role)

    cpdef set_action(self, int action):
        self._mitecom.set_action(action)

    cpdef set_state(self, int state):
        self._mitecom.set_state(state)

    cpdef set_relative_ball(self, int x, int y):
        self._mitecom.set_relative_ball(x, y)

    cpdef set_ball_time(self, int sec):
        self._mitecom.set_ball_time(sec)

    cpdef set_kickoff_offence_side(self, int side):
        self._mitecom.set_kickoff_offence_side(side)


    cpdef set_team_mate(self, intlist):
        x, y, tm3, tm4, tm5, tm6, tm7, tm8 = intlist
        self._mitecom.set_team_mate(x, y, tm3, tm4, tm5, tm6, tm7, tm8)

    cpdef set_opponent_robot(self, intlist):
        x, y, op3, op4, op5, op6, op7, op8 = intlist
        self._mitecom.set_opponent_robot(x, y, op3, op4, op5, op6, op7, op8)


    cpdef dict recv_data(self):
        self._mitecom.recieve_data()
        cdef const map[int,_TeamMateData*]* orig_data = self._mitecom.get_data()
        cdef map[int, _TeamMateData*] orig_data_map = deref(orig_data)
        cdef dict data = {}
        cdef TeamMateData newrobot
        for robot in orig_data_map:
            newrobot = TeamMateData()
            newrobot.set_team_mate_data(deref(robot.second))
            data[robot.first] = newrobot
        return data


# from mitecom-data.h

# robot is not doing anything or incapable of doing anything
STATE_INACTIVE                                  = 0
# the robot is ready to play or is playing already
STATE_ACTIVE                                    = 1
# The robot is penalized
STATE_PENALIZED                                 = 2

#  undefined action (if nothing else matches)
ACTION_UNDEFINED                             = 0
# robot is trying to position at a certain position
ACTION_POSITIONING                           = 1
# robot is trying to reach the ball
ACTION_GOING_TO_BALL                         = 2
# robot is in the possession of the ball (i.e. ball is in front)
# and is actively trying to move the ball into the opponent goal
# by e.g. dribbling or kicking ...
ACTION_TRYING_TO_SCORE                       = 3
# robot is waiting for an event (e.g. ball coming closer)
ACTION_WAITING                               = 4

# robot is not doing anything
ROLE_IDLING                                  = 0

# undefined role, no role or some weird stuff
ROLE_OTHER                                   = 1

# A striker is actively pursuing the ball.
ROLE_STRIKER                                 = 2

# A supporter is positioning itself close to the striker or the
# ball to be able to take over the striker role if necessary,
# and/or to block access to the ball or to be the first line of
# defense once the striker is removed/fails.
ROLE_SUPPORTER                               = 3

# A defender is positioning itself not too far away from the goal
# in order to defend the goal against opponent strikers
ROLE_DEFENDER                                = 4

# A goalie is positioned inside the penalty box, and the last line
# of defense. It may touch the ball, and it has special protection.
ROLE_GOALIE                                  = 5
