# -*- coding:utf-8 -*-
"""
GoToPositionIntelligent
^^^^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from bitbots_common.stackmachine import AbstractActionModule
from geometry_msgs.msg import Pose2D


class GoToBallIntelligent(AbstractActionModule):
    """
    Goes to the ball
    """
    def __init__(self):
        pass

    def perform(self, connector, reevaluate=False):
        p = Pose2D()
        p.x, p.y = connector.vision.get_ball_relative()
        connector.walking.pub_walking_objective.publish(p)
        #todo definierne, dass jetzt Pathfinding benutzt werden soll

