import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class GoalScoreRecently(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoalScoreRecently, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        """
        Determines whether we scored an goal in the last n seconds
        :param reevaluate:
        :return:
        """

        if self.blackboard.gamestate.get_seconds_since_own_goal().to_sec() < 2:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True
