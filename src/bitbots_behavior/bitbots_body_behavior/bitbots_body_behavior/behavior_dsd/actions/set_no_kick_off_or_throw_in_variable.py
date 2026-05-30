from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class SetNoKickOffOrThrowInVariable(AbstractActionElement):
    """
    Sets the no_kick_off_or_throw_in variable.
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.do_not_reevaluate()
        self.blackboard = blackboard

        self.blackboard.misc.kickoff_or_throwin_kick = parameters.get("value", None)

    def perform(self, reevaluate=False):
        self.pop()
