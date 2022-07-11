from dynamic_stack_decider import AbstractActionElement

from bitbots_blackboard.blackboard import BodyBlackboard



class ChangeDuty(AbstractActionElement):
    def __init__(self, blackboard: BodyBlackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

        self.duty = parameters.get('duty', None)

    def perform(self, reevaluate=False):
        self.blackboard.blackboard.duty = self.duty
        self.pop()
