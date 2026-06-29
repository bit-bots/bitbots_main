from typing import Optional

import numpy as np
from sensor_msgs.msg import Imu
from transforms3d.quaternions import qinverse, qmult, quat2mat

from handlers.handler import Handler


class OrientationHandler(Handler):
    """Provides ``motion_anchor_ori_b`` (6D): the orientation of the reference motion's
    anchor relative to the robot base, expressed in the robot base frame.

    The motion lives in its own world frame (arbitrary heading); the robot's IMU world
    frame is different. At motion start we capture an alignment quaternion so the
    reference frame-0 orientation coincides with the robot's current orientation. Then
    ``motion_anchor_ori_b`` starts at identity and only grows as the robot deviates from
    the reference, which makes the start heading irrelevant. This mirrors the sim2sim
    computation (quat_inv(robot) * motion_anchor), but with the start alignment added for
    deployment where robot-world != motion-world.
    """

    def __init__(self, node):
        self._node = node
        self._imu_data: Optional[Imu] = None
        self._align = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # identity until captured

        self._node.create_subscription(Imu, "imu/data", self._imu_callback, 10)

    def _imu_callback(self, msg: Imu) -> None:
        self._imu_data = msg

    def has_data(self) -> bool:
        return self._imu_data is not None

    def _robot_quat(self) -> np.ndarray:
        assert self._imu_data is not None
        q = self._imu_data.orientation
        return np.array([q.w, q.x, q.y, q.z], dtype=np.float64)

    def capture_alignment(self, ref_quat0: np.ndarray) -> None:
        """Lock the motion world frame to the robot's current orientation (call at start)."""
        self._align = qmult(self._robot_quat(), qinverse(np.asarray(ref_quat0, dtype=np.float64)))

    def get_motion_anchor_ori_b(self, ref_quat: np.ndarray) -> np.ndarray:
        q_ref_w = qmult(self._align, np.asarray(ref_quat, dtype=np.float64))  # ref in robot world
        q12 = qmult(qinverse(self._robot_quat()), q_ref_w)  # ref expressed in robot base frame
        rot = quat2mat(q12)  # (3, 3)
        return rot[:, :2].reshape(6).astype(np.float32)  # first two columns -> 6D
