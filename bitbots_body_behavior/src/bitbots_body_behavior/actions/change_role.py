from dynamic_stack_decider import AbstractActionElement
from humanoid_league_msgs.msg import Strategy


class ChangeRole(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

        self.role = parameters.get('role', None)

    def perform(self, reevaluate=False):
        self.blackboard.team_data.set_role(self.blackboard.team_data.roles[self.role])
        self.pop()
