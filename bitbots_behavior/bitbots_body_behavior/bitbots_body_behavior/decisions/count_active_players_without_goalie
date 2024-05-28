from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class CountActiveRobotsWithoutGoalie (AbstractDecisionElement):
    """
    Decides what kind of behaviour the robot performs
    """
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        nummber_of_active_teammates = self.blackboard.team_data.get_number_of_active_fieldplayers(False)
        self.publish_debug_data("Number of active Teammates", nummber_of_active_teammates)
        if nummber_of_active_teammates == 2:
            return "TWO"
        # in case we play with three robots we want rank two to defend
        elif nummber_of_active_teammates == 1 or nummber_of_active_teammates == 3:
            return "OTHER"
        else:
            # emergency fall back if something goes wrong
            self.blackboard.node.get_logger().warning("Rank to ball had some issues")
            return "FIRST"

    def get_reevaluate(self):
        return True
