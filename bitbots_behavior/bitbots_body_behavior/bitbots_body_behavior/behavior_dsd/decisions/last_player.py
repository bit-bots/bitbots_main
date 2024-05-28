from bitbots_blackboard.blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class LastPlayer(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        # Get nessesary data
        red_cards = self.blackboard.gamestate.get_red_cards()
        own_id = self.blackboard.misc.bot_id

        # Use generator comprehension to check if all red cards are true except our own
        if all(x for i, x in enumerate(red_cards) if i != own_id):
            return "YES"
        else:
            return "NO"

    def get_reevaluate(self):
        return True
