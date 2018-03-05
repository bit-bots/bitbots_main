"""
Search
^^^^^^
"""
from bitbots_stackmachine.abstract_action_module import AbstractActionModule
from humanoid_league_msgs.msg import HeadMode


class Search(AbstractActionModule):
    def perform(self, connector, reevaluate=False):
        # Tell the head that it should search for the ball
        head_mode_msg = HeadMode()
        head_mode_msg.headMode = HeadMode.BALL_MODE
        connector.head_pub.publish(head_mode_msg)
        self.pop()


class StopAndSearch(Search):
    def perform(self, connector, reevaluate=False):
        connector.walking.stop_walking()
        super(StopAndSearch, self).perform(connector, reevaluate)
