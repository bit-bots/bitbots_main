from bitbots_blackboard.blackboard import BodyBlackboard

from dynamic_stack_decider import AbstractActionElement


class ChangeRole(AbstractActionElement):
    blackboard: BodyBlackboard
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

        self.role = parameters.get('role', None)

    def perform(self, reevaluate=False):
        self.blackboard.team_data.set_role(self.blackboard.team_data.roles[self.role])
        self.pop()
