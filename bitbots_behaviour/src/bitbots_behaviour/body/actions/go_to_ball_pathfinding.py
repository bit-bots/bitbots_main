"""
GotToBallPathfinding
^^^^^^^^^^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from bitbots_common.stackmachine.abstract_action_module import AbstractActionModule
from geometry_msgs.msg import Pose2D


class GoToBallPathfinding(AbstractActionModule):
    """
    Goes to the ball
    """
    def perform(self, connector, reevaluate=False):
        p = Pose2D()
        p.x, p.y = connector.vision.get_ball_relative()
        connector.walking.pub_walking_objective.publish(p)
