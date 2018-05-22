"""
BallSeenGoalie
^^^^^^^^^^^^^^
"""
import rospy

from bitbots_stackmachine.abstract_decision_module import AbstractDecisionModule
from bitbots_body_behaviour.body.decisions.goalie.ball_dangerous import BallDangerous
from bitbots_body_behaviour.body.actions.search import Search


class BallSeenGoalie(AbstractDecisionModule):
    def perform(self, connector, reevaluate=False):

        if rospy.get_time() - connector.personal_model.ball_last_seen() < 2:
            return self.push(BallDangerous)
        else:
            return self.push(Search)

    def get_reevaluate(self):
        return True
