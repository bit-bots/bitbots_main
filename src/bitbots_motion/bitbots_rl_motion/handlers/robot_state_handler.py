from typing import Optional

from bitbots_msgs.msg import RobotControlState
from handlers.handler import Handler

WALKABLE_STATES = (
    RobotControlState.CONTROLLABLE,
    RobotControlState.WALKING,
)

KICKABLE_STATES = (
    RobotControlState.CONTROLLABLE,
    RobotControlState.KICKING,
)


class RobotStateHandler(Handler):
    def __init__(self, node):
        self._node = node
        self._robot_state: Optional[int] = None
        self._node.create_subscription(RobotControlState, "robot_state", self._callback, 1)

    def _callback(self, msg: RobotControlState):
        self._robot_state = msg.state

    def has_data(self):
        return self._robot_state is not None

    def is_walkable(self):
        return self._robot_state in WALKABLE_STATES

    def is_kickable(self):
        return self._robot_state in KICKABLE_STATES
