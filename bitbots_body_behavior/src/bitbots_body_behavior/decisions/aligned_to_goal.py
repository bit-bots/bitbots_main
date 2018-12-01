from bitbots_dsd.abstract_decision_element import AbstractDecisionElement


class AlignedToGoal(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(AlignedToGoal, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        # TODO: get current angle in map, compare to angle of goal, use threshold to determine if we are right
        return 'YES'
