import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class GoalSeen(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(GoalSeen, self).__init__(blackboard, dsd, parameters)
        self.goal_lost_time = self.blackboard.config['goal_lost_time']

    def perform(self, reevaluate=False):
        self.publish_debug_data("goal_seen_time", rospy.get_time() - self.blackboard.world_model.goal_last_seen())
        self.publish_debug_data("goal_x", self.blackboard.world_model.get_detection_based_goal_position_uv()[0])
        self.publish_debug_data("goal_y", self.blackboard.world_model.get_detection_based_goal_position_uv()[1])
        if rospy.get_time() - self.blackboard.world_model.goal_last_seen() < self.goal_lost_time:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True
