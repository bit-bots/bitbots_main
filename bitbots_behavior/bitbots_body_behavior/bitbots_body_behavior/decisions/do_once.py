from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class DoOnce(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super().__init__(blackboard, dsd, parameters)
        self.done = False

    def perform(self, reevaluate=False):
        if self.done:
            return "DONE"
        else:
            self.done = True
            return "NOT_DONE"

    def get_reevaluate(self):
        return False
