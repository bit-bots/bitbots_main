from dynamic_stack_decider.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode


class LookAtBall(AbstractActionElement):
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE)
        self.pop()


class LookAtBallPenalty(AbstractActionElement):
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE_PENALTY)
        self.pop()


class LookAtFieldFeatures(AbstractActionElement):
    def perform(self):
        self.blackboard.blackboard.set_head_duty(HeadMode.FIELD_FEATURES)
        self.pop()
