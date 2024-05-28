from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class CountActiveRobotsWithoutGoalie(AbstractDecisionElement):
    """
    Decides what kind of behaviour the robot performs
    """

    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        nummber_of_active_teammates = self.blackboard.team_data.get_number_of_active_fieldplayers(False)
        nummber_of_active_teammates += 1  # TODO move in function
        self.publish_debug_data("Number of active Teammates", nummber_of_active_teammates)
        if nummber_of_active_teammates == 1:
            return "ONE"
        elif nummber_of_active_teammates == 2:
            return "TWO"
        elif nummber_of_active_teammates == 3:
            return "THREE"
        elif nummber_of_active_teammates == 4:
            return "FOUR"
        else:
            # emergency fall back if something goes wrong
            self.blackboard.node.get_logger().error("Rank to ball had some issues")
            raise ValueError("Count active Robots encountered to many robots")

    def get_reevaluate(self):
        return True
