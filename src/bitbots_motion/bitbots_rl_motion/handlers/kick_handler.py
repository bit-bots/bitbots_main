from std_msgs.msg import Bool

from handlers.handler import Handler


class KickHandler(Handler):
    def __init__(self, node):
        self._node = node
        self._kick_active = False
        self._node.create_subscription(Bool, "rl_kick_active", self._callback, 1)

    def _callback(self, msg: Bool) -> None:
        self._kick_active = msg.data

    def has_data(self) -> bool:
        return True  # Non-blocking

    def is_active(self) -> bool:
        return self._kick_active
