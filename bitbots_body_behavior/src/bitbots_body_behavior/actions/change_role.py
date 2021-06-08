from dynamic_stack_decider import AbstractActionElement
from humanoid_league_msgs.msg import Strategy


class ChangeRole(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

        self.role = parameters.get('role', None)
        self.roles = {
            'striker': Strategy.ROLE_STRIKER,
            'supporter': Strategy.ROLE_SUPPORTER,
            'defender': Strategy.ROLE_DEFENDER,
            'other': Strategy.ROLE_OTHER,
            'goalie': Strategy.ROLE_GOALIE,
            'idle': Strategy.ROLE_IDLING
        }

    def perform(self, reevaluate=False):
        self.blackboard.team_data.set_role(self.roles[self.role])
        self.pop()
