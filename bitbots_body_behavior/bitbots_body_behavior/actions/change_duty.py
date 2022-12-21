from bitbots_blackboard.blackboard import BodyBlackboard

from dynamic_stack_decider import AbstractActionElement


class ChangeDuty(AbstractActionElement):
    blackboard: BodyBlackboard
    def __init__(self, blackboard, dsd, parameters: dict):
        super().__init__(blackboard, dsd, parameters)

        self.duty = parameters.get('duty', None)

    def perform(self, reevaluate=False):
        self.blackboard.blackboard.duty = self.duty
        self.pop()
