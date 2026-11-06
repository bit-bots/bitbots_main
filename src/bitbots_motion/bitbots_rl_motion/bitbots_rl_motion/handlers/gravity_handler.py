from typing import Optional

import numpy as np
from sensor_msgs.msg import Imu
from transforms3d.quaternions import quat2mat

from bitbots_rl_motion.handlers import Handler


class GravityHandler(Handler):
    def __init__(self, node):
        self._node = node

        self._imu_data: Optional[Imu] = None

        self._imu_sub = self._node.create_subscription(Imu, "imu/data", self._imu_callback, 10)

    # Callables
    def _imu_callback(self, msg: Imu):
        self._imu_data = msg

    def has_data(self) -> bool:
        return self._imu_data is not None

    def get_gravity(self) -> np.ndarray:
        """
        Returns the gravity vector in the robot's base frame computed from the IMU orientation.
        """
        assert self._imu_data is not None, "IMU data is not available yet"
        q = self._imu_data.orientation
        gravity = quat2mat([q.w, q.x, q.y, q.z]).T @ np.array([0, 0, -1], dtype=np.float32)
        return gravity
