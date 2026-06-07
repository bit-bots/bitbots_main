from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class SetNoSecondBallContactVariable(AbstractActionElement):
    """
    Sets the no_second_ball_contact variable.
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.do_not_reevaluate()
        self.blackboard = blackboard

        self.blackboard.misc.no_second_ball_contact = parameters.get("value", None)

    def perform(self, reevaluate=False):
        self.pop()
