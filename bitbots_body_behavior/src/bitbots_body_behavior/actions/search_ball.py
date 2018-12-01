from bitbots_dsd.abstract_action_element import AbstractActionElement
from humanoid_league_msgs.msg import HeadMode


class SearchBall(AbstractActionElement):
    def __init__(self, blackboard, dsd, parameters=None):
        super(SearchBall, self).__init__(blackboard, dsd, parameters)

    def perform(self, reevaluate=False):
        self.blackboard.blackboard.set_head_duty(HeadMode.BALL_MODE)
        # TODO turn around when no ball has been seen
