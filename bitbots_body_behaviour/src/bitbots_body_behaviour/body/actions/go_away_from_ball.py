# -*- coding:utf-8 -*-
"""
GotToBallPathfinding
^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from geometry_msgs.msg import Pose2D, Twist

from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class GoAwayFromBall(AbstractActionModule):
    """
    Goes to the ball
    """
    def perform(self, connector, reevaluate=False):
        t = Twist()
        x, y = connector.vision.get_ball_relative()
        if x > 0:
            xgo = -0.5
        else:
            xgo = 0.5
        if y > 0:
            ygo = -0.5
        else:
            ygo = 0.5
        connector.walking.start_walking_plain(xgo, 0, ygo)
