from typing import Optional

import numpy as np
from geometry_msgs.msg import Twist

from handlers.handler import Handler


class CommandHandler(Handler):
    def __init__(self, node):
        self._node = node
        self._cmd_vel: Optional[Twist] = None
        self._cmd_vel_sub = self._node.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)


    def get_command(self):
        assert self._cmd_vel is not None
        return np.array(
            [self._cmd_vel.linear.x, self._cmd_vel.linear.y, self._cmd_vel.angular.z],
            dtype=np.float32,
        )

    def get_command_with_stop(self):
        assert self._cmd_vel is not None
        stop_signal = float(self.get_stop_signal())
        return np.array(
            [self._cmd_vel.linear.x, self._cmd_vel.linear.y, self._cmd_vel.angular.z, stop_signal],
            dtype=np.float32,
        )

    def get_stop_signal(self) -> bool:
        assert self._cmd_vel is not None
        return self._cmd_vel.angular.x != 0.0

    def has_data(self):
        return self._cmd_vel is not None

    def _cmd_vel_callback(self, msg):
        self._cmd_vel = msg
