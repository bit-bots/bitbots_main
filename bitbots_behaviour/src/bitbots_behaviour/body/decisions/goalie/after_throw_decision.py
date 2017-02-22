"""
InGoal
^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from stackmachine.abstract_decision_module import AbstractDecisionModule

from body.actions.throw import MIDDLE
from body.decisions.goalie.turn_after_throw import TurnAfterThrow
from stackmachine.model import BodyConnector


class AfterThrowDecision(AbstractDecisionModule):
    """
    Decides how the robot will turn after it has thrown itself.
    """

    def __init__(self, connector: BodyConnector, _):
        super(AfterThrowDecision, self).__init__(connector)
        self.relocateTurn = config["Behaviour"]["Toggles"]["Goalie"]["relocateTurn"]
        self.anim_goalie_walkready = config["animations"]["motion"]["goalie-walkready"]

    def perform(self, connector: BodyConnector, reevaluate=False):
        richtung = connector.blackboard.get_throw_direction()
        if richtung == MIDDLE:
            connector.blackboard.delete_was_thrown()
            connector.animation.play_animation()

        if self.relocateTurn:
            return self.push(TurnAfterThrow)
        else:
            connector.blackboard.delete_was_thrown()
            return self.pop()
