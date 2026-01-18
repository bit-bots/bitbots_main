from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class ChangeRole(AbstractActionElement):
    """
    Changes the role of the robot in the team. This can be e.g. 'goalie'.
    It is used to e.g. determine the role position.
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters: dict):
        super().__init__(blackboard, dsd, parameters)

        self.role = parameters.get("role", None)

    def perform(self, reevaluate=False):
        self.blackboard.team_data.set_role(self.role)
        self.pop()
