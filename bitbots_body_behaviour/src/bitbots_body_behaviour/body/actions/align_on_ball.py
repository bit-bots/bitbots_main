"""
AlignOnBall
^^^^^^^^^^^

.. moduleauthor:: Martin Poppinga <1popping@informatik.uni-hamburg.de>

"""
from bitbots_stackmachine.abstract_action_module import AbstractActionModule
from bitbots_common.connector.connector import BodyConnector


class AlignOnBall(AbstractActionModule):
    def perform(self, connector: BodyConnector, reevaluate=False):
        connector.blackboard.schedule_ball_tracking()

        connector.walking.start_walking_plain(
            -1,
            0,
            self.sign(connector.world_model.get_ball_position_uv()[1] * 2)
        )
