import numpy as np
from bitbots_blackboard.body_blackboard import BodyBlackboard
from dynamic_stack_decider.abstract_action_element import AbstractActionElement


class WaitAfterKickOff(AbstractActionElement):
    """Logic for waiting after kick off."""

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.blackboard: BodyBlackboard

        self.wait_after_kick_time = self.blackboard.config["waiting_after_kick_time"]
        self.wait_after_kick_effective_radius = self.blackboard.config["waiting_after_kick_effective_radius"]
        self.wait_after_kick_bool = self.blackboard.config["waiting_after_kick_bool"]

    def perform(self, reevaluate=False) -> int:
        """If activated: Waits till a ball is out of the efffective zone or the waiting time is over."""
        ball_position_uv = self.blackboard.world_model.get_ball_position_uv()

        if self.wait_after_kick_bool:
            stop_time = self._node.get_clock().now() + self.wait_after_kick_time

            if self._node.get_clock().now() < stop_time:
                if np.square(self.wait_after_kick_effective) < np.square(ball_position_uv[0]) + np.square(
                    ball_position_uv[1]
                ):
                    return

                self._node.get_clock().sleep_for(0.1)
            return
