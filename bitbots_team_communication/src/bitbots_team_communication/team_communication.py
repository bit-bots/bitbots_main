# -*- coding:utf-8 -*-
import rospy
import time

from humanoid_league_msgs.msg import Role, Action, Position, MotionState, BallRelative, TeamData

from mitecom.mitecom import MiteCom, STATE_PENALIZED, ROLE_IDLING, ACTION_UNDEFINED,STATE_INACTIVE


class TeamCommunication(object):
    # todo docu
    # todo maybe add ball_relative_belife to mitecome to propagade the confidence value from the ROS message

    def __init__(self):
        rospy.init_node('bitbots_team_communication', anonymous=False)

        rospy.Subscriber("role", Role, self.role_callback)
        rospy.Subscriber("action", Action, self.action_callback)
        rospy.Subscriber("position", Position, self.position_callback)
        rospy.Subscriber("motion_state", MotionState, self.motion_state_callback)
        rospy.Subscriber("ball_relative", BallRelative, self.ball_callback)

        self.publisher = rospy.Publisher("/team_data", TeamData)

        self.port = rospy.get_param("/team_communication/port")
        self.mitecom_enabled = rospy.get_param("/team_communication/enabled")
        # todo this should be somehow global, maybe include it into humanoid_league_msgs
        self.team = rospy.get_param("/team_communication/team")
        self.player = rospy.get_param("/team_communication/player")
        #publishing rate in Hz
        self.rate = rospy.Rate(rospy.get_param("/team_communication/rate"))

        self.role = ROLE_IDLING
        self.action = ACTION_UNDEFINED
        self.state = STATE_INACTIVE
        self.absolute_x = 100  # todo: better start value
        self.absolute_y = 100
        self.absolute_orientation = 0
        self.absolute_belief = 0
        self.ball_relative_x = 100
        self.ball_relative_y = 0
        self.ball_certainty = 0

        self.avg_walking_speed = rospy.get_param("/team_communication/avg_walking_speed")
        self.time_to_position_at_ball = 0
        self.max_kicking_distance = 0

    def run(self):
        mitecom = MiteCom(self.port, self.team)
        mitecom.set_robot_id(self.player)
        while True:
            if not self.mitecom_enabled:
                # currently not active, we wait a bit
                rospy.sleep(1)
                continue

            mitecom.set_state(self.state)
            mitecom.set_role(self.role)
            if self.ball_certainty > 0 and not self.state == STATE_PENALIZED:
                mitecom.set_relative_ball(self.ball_relative_x, self.ball_relative_y)
                #todo find some solution for the time to get to the ball
                #mitecom.set_ball_time(self.get(DATA_KEY_BALL_TIME))

            #todo find some solution for additional data from the behaviour
            #if self.get(DATA_KEY_KICKOFF_OFFENSE_SIDE, 0) != 0:
            #    offence_side = self.get(DATA_KEY_KICKOFF_OFFENSE_SIDE)
            #    mitecom.set_kickoff_offence_side(offence_side)

            #todo check if this is optional data or not in mitecome
            if (not functools.reduce(lambda x, y: x and y,
                                     map(lambda e1, e2: e1 == e2,
                                         self.get(DATA_KEY_OPPONENT_ROBOT, [0, 0, 0, 0, 0, 0, 0, 0]),
                                         [0, 0, 0, 0, 0, 0, 0, 0])) and
                    (not self.penalty)):
                opponent_robot = self.get(DATA_KEY_OPPONENT_ROBOT)
                mitecom.set_opponent_robot(opponent_robot)

            if (not functools.reduce(lambda x, y: x and y,
                                     map(lambda e1, e2: e1 == e2,
                                         self.get(DATA_KEY_TEAM_MATE, [0, 0, 0, 0, 0, 0, 0, 0]),
                                         [0, 0, 0, 0, 0, 0, 0, 0]))) and \
                    (not self.penalty):
                team_mate = self.get(DATA_KEY_TEAM_MATE)
                mitecom.set_team_mate(team_mate)

            mitecom.send_data()

            data = mitecom.recv_data()
            if data != {}:
                self.publish_data(data)
            self.rate.sleep()

    def publish_data(self, data):
        message = TeamData()
        roles = []
        actions = []
        states = []
        absolute_xs = []
        absolute_ys = []
        absolute_orientations = []
        absolute_beliefs = []
        ball_relative_xs = []
        ball_relative_ys = []
        avg_walking_speeds = []
        time_to_position_at_balls = []
        max_kicking_distances = []

        for robid in data:
            rob = data[robid]
            roles[robid] = rob.get_role()
            actions[robid] = rob.get_action()
            states[robid] = rob.get_state()
            absolute_xs[robid] = rob.get_absolute_x()
            absolute_ys[robid] = rob.get_absolute_x()
            absolute_orientations[robid] = rob.getabsolute_orientation()
            absolute_beliefs[robid] = rob.get_absolute_belief()
            ball_relative_xs[robid] = rob.get_ball_relative_x()
            ball_relative_ys[robid] = rob.get_ball_relative_y()
            avg_walking_speeds[robid] = rob.get_avg_walking_speed()
            time_to_position_at_balls[robid] = rob.get_time_to_position_at_ball()
            max_kicking_distances[robid] = rob.get_max_kicking_distance()

        self.publisher.publish(message)

    def role_callback(self, msg):
        self.role = msg.role

    def action_callback(self, msg):
        self.action = msg.action

    def position_callback(self, msg):
        # todo check if conversion from mm to m
        self.absolute_x = msg.pose.x
        self.absolute_y = msg.pose.y
        self.absolute_orientation = msg.pose.theta
        # the scale is different in mitecome, so we have to transfer
        self.absolute_belief = int(msg.confidence * 255)

    def motion_state_callback(self, msg):
        state = msg.state
        if state == MotionState.PENALTY or state == MotionState.PENALTY_ANIMATION:
            self.state = STATE_PENALIZED
        elif state == MotionState.STARTUP or state == MotionState.SHUTDOWN or state == MotionState.RECORD:
            self.state = STATE_INACTIVE
        else:
            self.state = MotionState.STATE_ACTIVE

    def ball_callback(self, msg):
        # todo check if conversion from mm to m
        self.ball_relative_x = msg.ball_relative.x
        self.ball_relative_y = msg.ball_relative.y
        self.ball_certainty = msg.certainty
