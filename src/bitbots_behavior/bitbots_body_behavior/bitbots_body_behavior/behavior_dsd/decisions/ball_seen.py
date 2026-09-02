from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class BallSeen(AbstractDecisionElement):
    """Determines whether this robot observed the ball itself."""

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        if self.blackboard.world_model.ball_seen():
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True


class TeamBallSeen(AbstractDecisionElement):
    """
    Determines whether anybody in our team knows where the ball is.

    This is what tells the team that it has to search for the ball,
    even if this robot does not see the ball at the moment.
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        if self.blackboard.world_model.team_ball_seen():
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
