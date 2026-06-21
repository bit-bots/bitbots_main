from typing import Optional

import numpy as np
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform support)
import tf2_ros as tf2
from geometry_msgs.msg import PoseStamped
from ros2_numpy import numpify
from tf_transformations import euler_from_quaternion

from handlers.handler import Handler


class KickDirectionHandler(Handler):
    """
    Provides the desired kick direction as a unit vector in the ``base_link`` frame.

    The direction is requested by the behavior on the ``kick_direction`` topic. This handler is
    non-blocking (``has_data`` always returns ``True``) so that the kick node keeps ticking - and
    keeps its phase in sync with the walk policy - even while no kick is requested. Until a
    direction is received it defaults to straight ahead.
    """

    def __init__(self, node, tf_buffer):
        self._node = node
        self._tf_buffer = tf_buffer
        self._kick_direction: Optional[PoseStamped] = None
        self._node.create_subscription(PoseStamped, "kick_direction", self._callback, 1)

    def _callback(self, msg: PoseStamped):
        self._kick_direction = msg

    def has_data(self) -> bool:
        return True  # Non-blocking

    def get_kick_direction(self) -> np.ndarray:
        if self._kick_direction is None:
            return np.array([1.0, 0.0], dtype=np.float32)

        try:
            target_local = self._tf_buffer.transform(self._kick_direction, "base_link")
            yaw = euler_from_quaternion(numpify(target_local.pose.orientation))[2]
            kick_dir = np.array([np.cos(yaw), np.sin(yaw)], dtype=np.float32)
        except (tf2.ConnectivityException, tf2.LookupException, tf2.ExtrapolationException):
            self._node.get_logger().error("Could not transform kick direction", throttle_duration_sec=1.0)
            kick_dir = np.array([1.0, 0.0], dtype=np.float32)

        norm = np.linalg.norm(kick_dir)
        if norm > 0.0:
            kick_dir = kick_dir / norm
        return kick_dir
