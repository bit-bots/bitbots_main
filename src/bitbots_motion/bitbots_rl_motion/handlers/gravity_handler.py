from typing import Optional

import numpy as np
from sensor_msgs.msg import Imu
from transforms3d.quaternions import quat2mat
from transforms3d.euler import euler2mat

from handlers.handler import Handler


class GravityHandler(Handler):
    def __init__(self, node):
        self._node = node

        self._imu_data: Optional[Imu] = None
        self._roll_offset = self._node.get_parameter("gravity.roll_offset").value
        self._pitch_offset = self._node.get_parameter("gravity.pitch_offset").value
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
        gravity = quat2mat([q.w, q.x, q.y, q.z]).T @ euler2mat(self._roll_offset, self._pitch_offset, 0) @ np.array([0, 0, -1], dtype=np.float32)
        self._node.get_logger().warn(f"offset pitch: {self._pitch_offset}, offset roll: {self._roll_offset}, gravity: {gravity}")
        return gravity
