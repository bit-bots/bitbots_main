#!/usr/bin/env python
# -*- coding:utf-8 -*-
import math
import rospy
import time

from humanoid_league_msgs.msg import Role, Action, Position, MotionState, BallRelative, TeamData, Position2D, GoalRelative, ObstaclesRelative

from geometry_msgs.msg import Pose2D

from bitbots_team_communication.mitecom.mitecom import MiteCom, STATE_PENALIZED, ROLE_IDLING, ACTION_UNDEFINED, \
    STATE_INACTIVE


class TeamCommunication(object):
    # todo docu
    # todo oppgoal to goal
    # todo belief to confidence

    def __init__(self):
        rospy.init_node('bitbots_team_communication', anonymous=False)

        # -- Class variables
        self.role = ROLE_IDLING
        self.action = ACTION_UNDEFINED
        self.state = STATE_INACTIVE

        self.position_x = None
        self.position_y = None
        self.position_orientation = None
        self.position_belief = None

        self.ball_relative_x = None
        self.ball_relative_y = None
        self.ball_belief = None

        self.oppgoal_relative_x = None
        self.oppgoal_relative_y = None
        self.oppgoal_belief = None

        self.opponent_robots = None
        self.team_robots = None

        self.time_to_position_at_ball = None
        self.offensive_side = None

        # --- Params ---
        self.port = rospy.get_param("/team_communication/port")
        self.team = rospy.get_param("/team_communication/team")
        self.player = rospy.get_param("/team_communication/player")
        # publishing rate in Hz
        self.rate = rospy.Rate(rospy.get_param("/team_communication/rate"))
        self.avg_walking_speed = rospy.get_param("/team_communication/avg_walking_speed")
        self.max_kicking_distance = rospy.get_param("/team_communication/max_kicking_distance")

        # --Publishers and Subscribers ---
        self.publisher = rospy.Publisher("/team_data", TeamData, queue_size=10)

        rospy.Subscriber("role", Role, self.role_callback)
        rospy.Subscriber("action", Action, self.action_callback)
        rospy.Subscriber("position", Position, self.position_callback)
        rospy.Subscriber("motion_state", MotionState, self.motion_state_callback)
        rospy.Subscriber("ball_relative", BallRelative, self.ball_callback)
        rospy.Subscriber("goal_relative", GoalRelative, self.goal_callback)
        rospy.Subscriber("obstacle_relative", ObstaclesRelative, self.obstacle_callback)

    def run(self):
        mitecom = MiteCom(self.port, self.team)
        mitecom.set_robot_id(self.player)

        while not rospy.is_shutdown():
            #todo check if transmitting while beeing penalized is allowed

            # state
            mitecom.set_state(self.state)
            mitecom.set_action(self.action)
            mitecom.set_role(self.role)

            # position
            if self.position_belief is not None and self.position_belief > 0 and self.state == STATE_PENALIZED:
                mitecom.set_pos(self.position_x, self.position_y, self.position_orientation, self.position_belief)

            # ball
            if self.ball_belief is not None and self.ball_belief > 0 and not self.state == STATE_PENALIZED:
                mitecom.set_relative_ball(self.ball_relative_x, self.ball_relative_y)

            # opponent goal
            if self.oppgoal_belief is not None and self.oppgoal_belief > 0 and not self.state == STATE_PENALIZED:
                mitecom.set_opp_goal_relative(self.oppgoal_relative_x, self.oppgoal_relative_y, self.oppgoal_belief)

            # opponent robots
            if self.oppgoal_belief is not None:
                if len(self.opponent_robots) > 0:
                    mitecom.set_opponent_robot_a(self.opponent_robots[0].x, self.opponent_robots[0].y, self.opponent_robots[0].confidence)
                if len(self.opponent_robots) > 1:
                    mitecom.set_opponent_robot_a(self.opponent_robots[1].x, self.opponent_robots[1].x, self.opponent_robots[1].confidence)
                if len(self.opponent_robots) > 2:
                    mitecom.set_opponent_robot_a(self.opponent_robots[2].x, self.opponent_robots[2].x, self.opponent_robots[2].confidence)
                if len(self.opponent_robots) > 3:
                    mitecom.set_opponent_robot_a(self.opponent_robots[3].x, self.opponent_robots[3].x, self.opponent_robots[3].confidence)

            # team robots
            if self.oppgoal_belief is not None:
                if len(self.team_robots) > 0:
                    mitecom.set_team_robot_a(self.team_robots[0].x, self.team_robots[0].y,
                                                 self.team_robots[0].confidence)
                if len(self.team_robots) > 1:
                    mitecom.set_team_robot_a(self.team_robots[1].x, self.team_robots[1].x,
                                                 self.team_robots[1].confidence)
                if len(self.team_robots) > 2:
                    mitecom.set_team_robot_a(self.team_robots[2].x, self.team_robots[2].x,
                                                 self.team_robots[2].confidence)
                    
            # time to ball
            if self.time_to_position_at_ball is not None:
                mitecom.set_ball_time(self.time_to_position_at_ball)

            # strategy
            if self.offensive_side is not None
                mitecom.set_kickoff_offence_side(self.offensive_side)

            mitecom.send_data()

            data = mitecom.recv_data()
            if data != {}:
                self.publish_data(data)
            self.rate.sleep()

    def publish_data(self, data):
        ids = []
        roles = []
        actions = []
        states = []
        own_position = []
        own_position_beliefs = []
        ball_relative = []
        oppgoal_relative = []
        opponent_robot_a = []
        opponent_robot_b = []
        opponent_robot_c = []
        opponent_robot_d = []
        team_robot_a = []
        team_robot_b = []
        team_robot_c = []

        avg_walking_speeds = []
        time_to_position_at_balls = []
        max_kicking_distances = []

        offensive_side = []

        # iterate through all robots from which we received data
        for robid in data:
            ids.append(robid)
            rob = data[robid]

            roles.append(rob.get_role())
            actions.append(rob.get_action())
            states.append(rob.get_state())

            # own position
            pos_msg = Pose2D()
            pos_msg.x = rob.get_absolute_x() / 1000
            pos_msg.y = rob.get_absolute_y() / 1000
            pos_msg.theta = rob.getabsolute_orientation()
            own_position.append(pos_msg)
            own_position_beliefs.append(rob.get_absolute_belief() / 255)

            # ball
            ball_msg = Position2D
            ball_msg.x = rob.get_ball_relative_x() / 1000
            ball_msg.y = rob.get_ball_relative_y() / 1000
            ball_msg.confidence = rob.get_ball_belief() / 255
            ball_relative.append(ball_msg)

            # oppgoal
            oppgoal_msg = Position2D
            oppgoal_msg.x = rob.get_oppgoal_relative_x() / 1000
            oppgoal_msg.y = rob.get_oppgoal_relative_y() / 1000
            oppgoal_msg.confidence = rob.get_oppgoal_belief() / 255
            oppgoal_relative.append(oppgoal_msg)

            # opponent_robot_a
            opponent_robot_a_msg = Position2D
            opponent_robot_a_msg.x = rob.get_opponent_robot_a_x() / 1000
            opponent_robot_a_msg.y = rob.get_opponent_robot_a_y() / 1000
            opponent_robot_a_msg.confidence = rob.get_opponent_robot_a_belief() / 255
            opponent_robot_a.append(opponent_robot_a_msg)

            # opponent_robot_b
            opponent_robot_b_msg = Position2D
            opponent_robot_b_msg.x = rob.get_opponent_robot_b_x() / 1000
            opponent_robot_b_msg.y = rob.get_opponent_robot_b_y() / 1000
            opponent_robot_b_msg.confidence = rob.get_opponent_robot_b_belief() / 255
            opponent_robot_b.append(opponent_robot_b_msg)

            # opponent_robot_c
            opponent_robot_c_msg = Position2D
            opponent_robot_c_msg.x = rob.get_opponent_robot_c_x() / 1000
            opponent_robot_c_msg.y = rob.get_opponent_robot_c_y() / 1000
            opponent_robot_c_msg.confidence = rob.get_opponent_robot_c_belief() / 255
            opponent_robot_c.append(opponent_robot_c_msg)

            # opponent_robot_d
            opponent_robot_d_msg = Position2D
            opponent_robot_d_msg.x = rob.get_opponent_robot_d_x() / 1000
            opponent_robot_d_msg.y = rob.get_opponent_robot_d_y() / 1000
            opponent_robot_d_msg.confidence = rob.get_opponent_robot_d_belief() / 255
            opponent_robot_d.append(opponent_robot_d_msg)

            # team_robot_a
            team_robot_a_msg = Position2D
            team_robot_a_msg.x = rob.get_team_robot_a_x() / 1000
            team_robot_a_msg.y = rob.get_team_robot_a_y() / 1000
            team_robot_a_msg.confidence = rob.get_team_robot_a_belief() / 255
            team_robot_a.append(team_robot_a_msg)

            # team_robot_b
            team_robot_b_msg = Position2D
            team_robot_b_msg.x = rob.get_team_robot_b_x() / 1000
            team_robot_b_msg.y = rob.get_team_robot_b_y() / 1000
            team_robot_b_msg.confidence = rob.get_team_robot_b_belief() / 255
            team_robot_b.append(team_robot_b_msg)

            # team_robot_c
            team_robot_c_msg = Position2D
            team_robot_c_msg.x = rob.get_team_robot_c_x() / 1000
            team_robot_c_msg.y = rob.get_team_robot_c_y() / 1000
            team_robot_c_msg.confidence = rob.get_team_robot_c_belief() / 255
            team_robot_c.append(team_robot_c_msg)

            avg_walking_speeds.append(rob.get_avg_walking_speed())
            time_to_position_at_balls.append(rob.get_time_to_position_at_ball())
            max_kicking_distances.append(rob.get_max_kicking_distance())

            offensive_side.append(rob.get_offensive_side())

        # build message
        message = TeamData()
        message.header.stamp = rospy.Time.now()

        message.robot_ids = ids

        message.role = roles
        message.action = actions
        message.state = states

        message.own_position = own_position
        message.own_position_confidence = own_position_beliefs

        message.ball_relative = ball_relative

        message.oppgoal_relative = oppgoal_relative

        message.opponent_robot_a = opponent_robot_a
        message.opponent_robot_b = opponent_robot_b
        message.opponent_robot_c = opponent_robot_c
        message.opponent_robot_d = opponent_robot_d

        message.team_robot_a = team_robot_a
        message.team_robot_b = team_robot_b
        message.team_robot_c = team_robot_c

        message.avg_walking_speed = avg_walking_speeds
        message.time_to_position_at_ball = time_to_position_at_balls
        message.max_kicking_distance = max_kicking_distances

        message.offensive_side = offensive_side

        self.publisher.publish(message)

    def role_callback(self, msg):
        self.role = msg.role

    def action_callback(self, msg):
        self.action = msg.action

    def motion_state_callback(self, msg):
        state = msg.state
        if state == MotionState.PENALTY or state == MotionState.PENALTY_ANIMATION:
            self.state = STATE_PENALIZED
        elif state == MotionState.STARTUP or state == MotionState.SHUTDOWN or state == MotionState.RECORD:
            self.state = STATE_INACTIVE
        else:
            self.state = MotionState.STATE_ACTIVE

    def position_callback(self, msg):
        # conversion from m (ROS message) to mm (mitecom)
        self.position_x = int(msg.pose.x * 1000)
        self.position_y = int(msg.pose.y * 1000)
        self.position_orientation = int(msg.pose.theta)
        # the scale is different in mitecom, so we have to transfer from 0...1 to 0...255
        self.position_belief = int(msg.confidence * 255)

    def ball_callback(self, msg):
        # conversion from m (ROS message) to mm (mitecom)
        self.ball_relative_x = int(msg.ball_relative.x * 1000)
        self.ball_relative_y = int(msg.ball_relative.y * 1000)
        # the scale is different in mitecom, so we have to transfer from 0...1 to 0...255
        self.ball_belief = int(msg.confidence * 255)
        # use pythagoras to compute time to ball
        self.time_to_position_at_ball = math.sqrt(self.ball_relative_x**2 + self.ball_relative_y**2)/self.avg_walking_speed

    def goal_callback(self, msg):
        #todo check if this is a good way to compute the position
        post_a = msg.positions[0]
        post_b = msg.positions[1]
        self.oppgoal_relative_x = (post_a.x - post_b.x) / 2 + post_a.x
        self.oppgoal_relative_y = (post_a.y - post_b.y) / 2 + post_a.y
        self.oppgoal_belief = msg.confidence

    def obstacle_callback(self, msg):
        # todo get own team color
        team_color = CYAN_ROBOT
        opponent_color = MAGENTA_ROBOT
        self.opponent_robots = []
        self.team_robots = []
        for obstacle in msg.obstacles:
            #only take obstacles that are team mates or opponents
            if obstacle.type == team_color:
                self.team_robots.append(obstacle.position)
            elif obstacle.type == opponent_color:
                self.opponent_robots.append(obstacle.position)

if __name__ == "__main__":
    print("Starting Team Communication")
    TeamCommunication()

