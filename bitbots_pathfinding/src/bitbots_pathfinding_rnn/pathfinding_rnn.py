#!/usr/bin/env python
# -*- coding:utf-8 -*-

import time
import math
import pickle
import rospy
from bitbots_pathfinding_rnn.potential_field import PotentialMap
from bitbots_pathfinding_rnn.rnn import Network
from collections import namedtuple
from typing import List, Tuple

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from humanoid_league_msgs.msg import GoalRelative
from humanoid_league_msgs.msg import ObstaclesRelative


class Pathfinding:
    def __init__(self):
        rospy.logdebug("Init Pathfinding")
        rospy.init_node("bitbots_pathfinding_rnn")
        self.conf_align_to_goal = rospy.get_param("/pathfinding/conf_align_to_goal")
        self.conf_refreshRate = rospy.get_param("/pathfinding/refreshRate")

        self.publish_walking = rospy.Publisher("/cmd_vel", Twist)

        rospy.Subscriber("/navigation_goal", Pose2D, self._update_naviagtiongoal)
        rospy.Subscriber("/obstacles_relative", ObstaclesRelative, self._update_obstacle)
        rospy.Subscriber("/goal_relative", GoalRelative, self._update_orientationobjective)  # TODO Aus localisation holen oder netz ändern, sodass schon im verhalten und dann pose relvandt

        self.last_vel = namedtuple("LastVel", ["forward", "turn", "sideward"])
        self.goalpos = namedtuple("GoalPosition", ["x", "y"])
        self.goalobjective = namedtuple("GoalObjective", ["x", "y"])
        self.obstacles = []

        self.network_path = rospy.get_param("", "")
        self.network = pickle.load(open(self.network_path, "r"))

        r = rospy.Rate(self.conf_refreshRate)

        while not rospy.is_shutdown():
            self.perform()
            r.sleep()

    def perform(self):
        # Slow down if near to position
        distancefactor = 1 if ((self.goalpos.x ** 2 + self.goalpos.y ** 2) ** 0.5) > 200 else 0.4

        # Neuronal net computes parameters for walking
        walk_parameters = self.network.compute([self.goalpos.x,
                                                self.goalpos.y,
                                                self.goalobjective.x,
                                                self.goalobjective.y,
                                                self.last_vel.forward,
                                                self.last_vel.turn,
                                                self.last_vel.sideward])

        potential_map = PotentialMap((len(self.obstacles), 0))
        f, t, s = potential_map.compute(self.obstacles)

        # put values together and take some factors
        tres = lambda x: 0.0 if abs(x) < 0.75 else x
        sig = lambda x: x / (1 + abs(x))
        turn = tres(walk_parameters[1] * 5.0)
        forward = ((tres(((walk_parameters[0]) + (f * 0.5)) * distancefactor)) * 5) + abs(turn / 4)

        side = sig(tres(((walk_parameters[2]) + s) * 0.3)) * 2.5

        rospy.logdebug(3, "#   F:" + str(forward) + "   #  T:" + str(turn) + "   #   S:" + str(side))

        self.last_vel.forward = forward
        self.last_vel.turn = turn
        self.last_vel.sideward = side

        msg = Twist()
        msg.linear = Vector3(forward, side, 0)
        msg.angular = Vector3(0, 0, turn)
        self.publish_walking.publish(msg)

    def _update_naviagtiongoal(self, pos: Pose2D):
        self.goalpos = (pos.x, pos.y)

    def _update_obstacle(self, obs: ObstaclesRelative):
        olist = [(o.position.x, o.position.y) for o in obs.obstacles]
        self.obstacles = olist
        self.obstacles.append(self.goalpos)

    def _update_orientationobjective(self, go: GoalRelative):
        if self.conf_align_to_goal:
            self.goalobjective = (go.positions[0].x, go.positions[0].y)
        else:
            self.goalobjective = (0, 0)

if __name__ == "__main__":
    Pathfinding()
