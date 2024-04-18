from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class RobotInOwnPercentOfField(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.percent = parameters["p"]

    def perform(self, reevaluate=False):
        """
        Determines whether the robot is in the given percentage of the field towards the own goal.
        :param reevaluate:
        :return:
        """
        robot_position = self.blackboard.world_model.get_current_position(self)
        # calculate the x value of the boundary of the defensive area
        defensive_x = ((self.percent / 100.0) * self.blackboard.world_model.field_length) - (
            self.blackboard.world_model.field_length / 2.0
        )
        if robot_position[0] <= defensive_x:
            return "YES"
        return "NO"

    def get_reevaluate(self):
        return True
