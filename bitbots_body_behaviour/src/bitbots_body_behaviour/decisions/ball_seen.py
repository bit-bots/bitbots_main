"""
BallSeen
^^^^^^^^

.. moduleauthor:: Martin Poppinga <robocup@poppinga.xyz>

"""

import rospy
from bitbots_dsd.abstract_decision_element import AbstractDecisionElement


class AbstractBallSeen(AbstractDecisionElement):
    """
    Decides if the ball was seen rspectively if the information is authentic enough.
    """

    def __init__(self, blackboard, _):
        super(AbstractBallSeen, self).__init__(blackboard)
        self.max_ball_time = blackboard.config["Body"]["Common"]["maxBallTime"]

    def perform(self, connector, reevaluate=False):
        if (rospy.get_time() - connector.world_model.ball_last_seen()) < self.max_ball_time:
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return True

    def _register(self):
        return ["YES", "NO"]


class BallSeen(AbstractBallSeen):
    pass
