from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class CountActiveRobotsWithoutGoalie(AbstractDecisionElement):
    """
    Decides what kind of behavior the robot performs
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        number_of_active_teammates = self.blackboard.team_data.get_number_of_active_field_players(False)
        self.publish_debug_data("Number of active Teammates", number_of_active_teammates)
        if number_of_active_teammates == 0:
            return "ZERO"
        elif number_of_active_teammates == 1:
            return "ONE"
        elif number_of_active_teammates == 2:
            return "TWO"
        elif number_of_active_teammates == 3:
            return "THREE"
        else:
            # emergency fall back if something goes wrong
            self.blackboard.node.get_logger().error("Rank to ball had some issues")
            raise ValueError("Count active Robots encountered to many robots")

    def get_reevaluate(self):
        return True
