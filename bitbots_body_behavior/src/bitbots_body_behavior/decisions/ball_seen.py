import rospy

from dynamic_stack_decider.abstract_decision_element import AbstractDecisionElement


class BallSeen(AbstractDecisionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(BallSeen, self).__init__(blackboard, dsd, parameters)
        self.ball_lost_time = rospy.Duration.from_sec(self.blackboard.config['ball_lost_time'])

    def perform(self, reevaluate=False):
        if rospy.Time.now() - self.blackboard.world_model.ball_last_seen() < self.ball_lost_time:
            return 'YES'
        return 'NO'

    def get_reevaluate(self):
        return True
