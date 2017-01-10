"""
FocusBall
^^^^^^^^^^
"""
from bitbots_common.stackmachine import AbstractActionModule


class FocusBall(AbstractActionModule):
    def perform(self, connector, reevaluate=False):
        connector.blackboard_capsule().schedule_ball_tracking()
        return self.pop()
