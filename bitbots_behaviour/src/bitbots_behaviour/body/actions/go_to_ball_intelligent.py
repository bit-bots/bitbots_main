# -*- coding:utf-8 -*-
"""
GoToPositionIntelligent
^^^^^^^^^^^^^^^^^^^^^^^

This module goes to a position by using a neural network. It can got to the ball, the duty positon or a given position.

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

history:

07.04.15: Changed to GoToPosition (Marc Bestmann)
"""
import time
import math
from bitbots.modules.abstract.abstract_module import debug_m
from bitbots.modules.abstract.abstract_action_module import AbstractActionModule
from bitbots.util import get_config
from bitbots.util.potentialfield import PotentialMap

config = get_config()


class AbstractGoToPositionIntelligent(AbstractActionModule):
    def __init__(self, _):
        self.align_to_goal = config["Behaviour"]["Toggles"]["Fieldie"]["alignToGoal"]
        self.toggle_track_both = config["Behaviour"]["Toggles"]["Fieldie"]["trackBoth"]
        self.focus_ball_distance = config["Behaviour"]["Fieldie"]["focusBallDistance"]
        self.f = 0
        self.t = 0
        self.s = 0
        self.last_f = 0
        self.last_t = 0
        self.last_s = 0
        self.last_iteration = time.time()

    def go_to_pos(self, connector, pu, pv, additional_obstacles, align_to_goal):

        # only run approx. 3 times a second:

        if time.time() - self.last_iteration > 0.33:
            network = connector.blackboard_capsule().get_pathfinding_net()
            self.last_iteration = time.time()

            if align_to_goal:
                gu, gv = connector.filtered_vision_capsule().get_local_goal_model_opp_goal()
            else:
                gu, gv = 0, 0

            # Slow down if near to position
            distancefactor = 1 if ((pu ** 2 + pv ** 2) ** 0.5) > 200 else 0.4

            # Neuronal net computes parameters for walking
            walk_parameters = network.compute([pu, pv, gu, gv, self.f, self.t, self.s])

            obstacles = []
            obstacles.extend(connector.raw_vision_capsule().get_horizon_obstacles())
            obstacles.extend(additional_obstacles)

            potential_map = PotentialMap((len(obstacles), 0))
            self.f, self.t, self.s = 0,0,0 # Bpotential_map.compute(obstacles)

            # put values together and take some factors
            tres = lambda x: 0.0 if abs(x) < 0.75 else x
            sig = lambda x: x / (1 + abs(x))
            turn = tres(walk_parameters[1] * 5.0)
            forward = ((tres(((walk_parameters[0]) + (self.f * 0.5)) * distancefactor)) * 5 ) + abs(turn / 4)

            side = sig(tres(((walk_parameters[2]) + self.s) * 0.3)) * 2.5

            #print(3, "#   F:" + str(forward) +  "   #  T:" + str(turn) + "   #   S:" + str(side))

            self.last_f = forward
            self.last_t = turn
            self.last_s = side

        else:
            forward = self.last_f
            turn = self.last_t
            side = self.last_s
        # start walking
        connector.walking_capsule().start_walking_plain(forward, turn, side)


class GoToBallIntelligent(AbstractGoToPositionIntelligent):
    """
    Goes to the ball
    """

    def perform(self, connector, reevaluate=False):

        # Get ball data
        bu, bv = connector.filtered_vision_capsule().get_local_goal_model_ball()
        bdist = math.sqrt(bu ** 2 + bv ** 2)

        # Look to the Ball
        if self.toggle_track_both and bdist > self.focus_ball_distance:
            connector.blackboard_capsule().schedule_both_tracking()
        else:
            connector.blackboard_capsule().schedule_ball_tracking()

        # The Ball is an obstacle, the Obstacles also and please avoid them
        obstacles = [
            [bu, bv]
        ]

        # use the value from the config
        align_to_goal = self.align_to_goal

        self.go_to_pos(connector, bu, bv, obstacles, align_to_goal)


class GoToPositionIntelligent(AbstractGoToPositionIntelligent):
    """
        Goes to position. Called with args: position, alignToGoal?
    """

    def __init__(self, args=None):
        super(GoToPositionIntelligent, self).__init__(None)
        if len(args) == 2:
            self.position = args[0]
            self.align = args[1]
        else:
            self.align = False
            self.position = args[0]

        self.track_goal = config["Behaviour"]["Toggles"]["Head"]["track_goal"]

    def perform(self, connector, reevaluate=False):

        # get the u and v values to position

        u, v = connector.world_model_capsule().get_uv_from_xy(self.position[0], self.position[1])

        # print "u:", u, " v: ", v

        if u is None:
            debug_m(3, "No position information")
            return
        if self.track_goal:
            connector.blackboard_capsule().schedule_enemy_goal_tracking()
        else:
            connector.blackboard_capsule().schedule_both_tracking()

        # there are no addional obstacles
        obstacles = []

        self.go_to_pos(connector, u, v, obstacles, self.align)
