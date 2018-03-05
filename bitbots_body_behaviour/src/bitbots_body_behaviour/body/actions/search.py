"""
Search
^^^^^^
"""
from bitbots_stackmachine.abstract_action_module import AbstractActionModule
from humanoid_league_msgs import HeadMode


class Search(AbstractActionModule):
    def perform(self, connector, reevaluate=False):
        # We do nothing here, the head is searching.
        connector.blackboard.schedule_ball_tracking()
        # Tell the head that it should search for the ball
        ball_mode_msg = HeadMode()
        ball_mode_msg.headMode = HeadMode.BALL_MODE
        connector.head_pub.publish(ball_mode_msg)
        self.pop()


class StopAndSearch(Search):
    def perform(self, connector, reevaluate=False):
        connector.walking.stop_walking()
        super(StopAndSearch, self).perform(connector, reevaluate)
