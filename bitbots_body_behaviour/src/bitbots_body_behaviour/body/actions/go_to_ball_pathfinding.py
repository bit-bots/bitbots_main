"""
GotToBallPathfinding
^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
import math

import rospy

from bitbots_common.connector.connector import BodyConnector
from bitbots_pathfinding.potential_field import PotentialMap


from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class GoToBallPathfinding(AbstractActionModule):
    """
    Goes to the ball
    """

    def __init__(self, _, connector: BodyConnector):
        super().__init__(connector)
        self.align_to_goal = None
        self.toggle_track_both = None
        self.focus_ball_distance = None
        self.f = 0
        self.t = 0
        self.s = 0
        self.last_f = 0
        self.last_t = 0
        self.last_s = 0
        self.last_iteration = rospy.get_time()

    def perform(self, connector: BodyConnector, reevaluate=False):
        self.align_to_goal = connector.config["Body"]["Toggles"]["Fieldie"]["alignToGoal"]
        self.toggle_track_both = connector.config["Body"]["Toggles"]["Fieldie"]["trackBoth"]
        self.focus_ball_distance = connector.config["Body"]["Fieldie"]["focusBallDistance"]

        # Get ball data
        bu, bv = connector.vision.get_ball_relative()
        bdist = math.sqrt(bu ** 2 + bv ** 2)

        # Look to the Ball
        if self.toggle_track_both and bdist > self.focus_ball_distance:
            connector.blackboard.set_head_duty("BALL_GOAL_TRACKING")
        else:
            connector.blackboard.set_head_duty("BALL_MODE")

        # The Ball is an obstacle, the Obstacles also and please avoid them
        obstacles = [
            [bu, bv]
        ]

        # use the value from the config
        align_to_goal = self.align_to_goal

        self.go_to_pos(connector, bu, bv, obstacles, align_to_goal)

    def go_to_pos(self, connector: BodyConnector, pu, pv, additional_obstacles, align_to_goal):

        # only run approx. 3 times a second:

        if rospy.get_time() - self.last_iteration > 0.33:
            network = connector.blackboard.get_pathfinding_net()
            self.last_iteration = rospy.get_time()

            if False and align_to_goal:
                pass
                # gu, gv = connector.vision.get_local_goal_model_opp_goal()
            else:
                gu, gv = 0, 0

            # Slow down if near to position
            distancefactor = 1 if ((pu ** 2 + pv ** 2) ** 0.5) > 200 else 0.4

            # Neuronal net computes parameters for walking
            walk_parameters = network.compute([pu, pv, gu, gv, self.f, self.t, self.s])

            obstacles = []
            #obstacles.extend(connector.vision.get_obstacle_info())
            obstacles.extend(additional_obstacles)

            potential_map = PotentialMap((len(obstacles), 0))
            self.f, self.t, self.s = potential_map.compute(obstacles)

            # put values together and take some factors
            tres = lambda x: 0.0 if abs(x) < 0.75 else x
            sig = lambda x: x / (1 + abs(x))
            turn = tres(walk_parameters[1] * 5.0)
            forward = ((tres(((walk_parameters[0]) + (self.f * 0.5)) * distancefactor)) * 5) + abs(turn / 4)

            side = sig(tres(((walk_parameters[2]) + self.s) * 0.3)) * 2.5

            self.last_f = forward
            self.last_t = turn
            self.last_s = side

        else:
            forward = self.last_f
            turn = self.last_t
            side = self.last_s
        # start walking
        connector.walking.start_walking_plain(forward/1000, turn/1000, side/1000)
