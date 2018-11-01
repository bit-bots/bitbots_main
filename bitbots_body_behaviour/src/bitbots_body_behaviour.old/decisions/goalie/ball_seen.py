"""
BallSeenGoalie
^^^^^^^^^^^^^^
"""
import rospy

from bitbots_stackmachine.abstract_decision_element import AbstractDecisionElement
from bitbots_body_behaviour.decisions.goalie.ball_dangerous import BallDangerous
from bitbots_body_behaviour.actions.search import Search


class BallSeenGoalie(AbstractDecisionElement):
    def perform(self, connector, reevaluate=False):

        if rospy.get_time() - connector.world_model.ball_last_seen() < 2:
            return self.push(BallDangerous)
        else:
            return self.push(Search)

    def get_reevaluate(self):
        return True
