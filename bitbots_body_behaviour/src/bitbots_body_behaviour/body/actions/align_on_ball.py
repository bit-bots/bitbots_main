"""
AlignOnBall
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from bitbots_stackmachine.abstract_action_module import AbstractActionModule
from bitbots_common.connector.connector import BodyConnector
from humanoid_league_msgs.msg import HeadMode


class AlignOnBall(AbstractActionModule):
    def perform(self, connector: BodyConnector, reevaluate=False):
        head_mode_msg = HeadMode()
        head_mode_msg.headMode = HeadMode.BALL_MODE
        connector.head_pub.publish(head_mode_msg)

        connector.walking.start_walking_plain(
            -1,
            0,
            self.sign(connector.vision.get_ball_relative()[1] * 2)
        )
