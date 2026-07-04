from std_msgs.msg import Bool

from bitbots_rl_motion.handlers import Handler


class DribbleActiveHandler(Handler):
    """Tracks the dribble start/stop flag published by the behavior.

    ``rl_dribble_active`` switches the dribble policy on and off. Used by the
    dribble node to gate its own execution and by the walk node to yield while
    the dribble policy is in control of the walking motor goals.
    """

    def __init__(self, node):
        self._node = node
        self._active = False
        self._node.create_subscription(Bool, "rl_dribble_active", self._callback, 1)

    def _callback(self, msg: Bool) -> None:
        self._active = msg.data

    def has_data(self) -> bool:
        return True  # Non-blocking

    def is_active(self) -> bool:
        return self._active
