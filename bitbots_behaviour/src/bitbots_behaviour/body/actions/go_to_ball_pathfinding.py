"""
GotToBallPathfinding
^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from geometry_msgs.msg import Pose2D

from bitbots_stackmachine.abstract_action_module import AbstractActionModule


class GoToBallPathfinding(AbstractActionModule):
    """
    Goes to the ball
    """
    def perform(self, connector, reevaluate=False):
        p = Pose2D()
        p.x, p.y = connector.vision.get_ball_relative()
        connector.walking.pub_walking_objective.publish(p)
