import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class GoalSeen(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoalSeen, self).__init__(blackboard, dsd, parameters)
        self.goal_lost_time = self.blackboard.config['goal_lost_time']

    def perform(self, reevaluate=False):
        if rospy.get_time() - self.blackboard.world_model.goal_last_seen() < self.goal_lost_time:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True
