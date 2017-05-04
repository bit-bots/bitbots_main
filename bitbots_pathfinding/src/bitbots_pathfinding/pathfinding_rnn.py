#!/usr/bin/env python3
import os
import pickle
import threading
from collections import namedtuple
from typing import Tuple

import rospy
from bitbots_pathfinding.potential_field import PotentialMap

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from humanoid_league_msgs.msg import GoalRelative
from humanoid_league_msgs.msg import ObstaclesRelative


vel = namedtuple("LastVel", ["forward", "turn", "sideward"])
point = namedtuple("Point", ["x", "y"])


class Pathfinding:
    def __init__(self):
        rospy.logdebug("Init Pathfinding")
        rospy.init_node("bitbots_pathfinding_rnn")
        self.conf_align_to_goal = rospy.get_param("pathfinding/align_to_goal")
        self.conf_refreshRate = rospy.get_param("pathfinding/refreshRate")

        self.publish_walking = rospy.Publisher("cmd_vel", Twist, queue_size=5)

        rospy.Subscriber("navigation_goal", Pose2D, self._update_naviagtiongoal)
        rospy.Subscriber("obstacles_relative", ObstaclesRelative, self._update_obstacle)
        rospy.Subscriber("goal_relative", GoalRelative,
                         self._update_orientationobjective)  # TODO Aus localisation holen oder netz ändern, sodass schon im verhalten und dann pose relvandt

        self.last_vel = vel(0, 0, 0)
        self.goalpos = None  # type: point
        self.goalobjective = point(0, 0)
        self.obstacles = []

        self.lock = threading.Lock()

        self.network_path = os.path.join(rospy.get_param("pathfinding/runpath"),
                                         rospy.get_param("pathfinding/network_path"))
        self.network = pickle.load(open(self.network_path, "rb"))

        r = rospy.Rate(self.conf_refreshRate)

        while not rospy.is_shutdown():
            if self.goalpos:  # New data avaliable
                with self.lock:
                    self.perform()
                    self.goalpos = None
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
        turn = -walk_parameters[1] / 2
        forward = ((walk_parameters[0] + f) / 2) * distancefactor

        side = (walk_parameters[2] + s) / 2

        rospy.logdebug("#   F:" + str(forward) + "   #  T:" + str(turn) + "   #   S:" + str(side))

        self.last_vel = vel(forward, turn, side)

        msg = Twist()
        msg.linear = Vector3(forward, side, 0)
        msg.angular = Vector3(0, 0, turn)
        self.publish_walking.publish(msg)

    def _update_naviagtiongoal(self, pos: Pose2D):
        with self.lock:
            self.goalpos = point(pos.x, pos.y)

    def _update_obstacle(self, obs: ObstaclesRelative):
        olist = [(o.position.x, o.position.y) for o in obs.obstacles]
        with self.lock:
            self.obstacles = olist
            self.obstacles.append(self.goalpos)

    def _update_orientationobjective(self, go: GoalRelative):
        with self.lock:
            if self.conf_align_to_goal:
                self.goalobjective = point(go.positions[0].x, go.positions[0].y)
            else:
                self.goalobjective = point(0, 0)

if __name__ == "__main__":
    Pathfinding()
