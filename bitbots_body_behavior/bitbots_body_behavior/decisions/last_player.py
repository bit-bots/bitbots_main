from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement

from bitbots_blackboard.blackboard import BodyBlackboard


class LastPlayer(AbstractDecisionElement):
    blackboard: BodyBlackboard

    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        red_cards = self.blackboard.gamestate.get_red_cards()
        own_id = self.blackboard.misc.bot_id

        # iterate through all red card states except the own one
        for i in range(len(red_cards)):
            if i != own_id:
                if not red_cards[i]:
                    return "NO"
        return "YES"

    def get_reevaluate(self):
        return True
