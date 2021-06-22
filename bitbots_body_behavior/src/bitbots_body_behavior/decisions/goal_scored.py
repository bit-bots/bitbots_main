import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class GoalScoreRecently(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoalScoreRecently, self).__init__(blackboard, dsd, parameters)
        self.time = parameters.get('time', 2)

    def perform(self, reevaluate=False):
        """
        Determines whether we scored an goal in the last n seconds
        :param reevaluate:
        :return:
        """

        if self.blackboard.gamestate.get_seconds_since_own_goal() < self.time:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True
