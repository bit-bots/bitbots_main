from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class IsPenaltyShootRobot(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.own_id = self.blackboard.team_data.bot_id

    def perform(self, reevaluate=False):
        """
        Determines if the robot is the one that should do something during penalty shoot out
        """
        if self.own_id == 1:
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return True
