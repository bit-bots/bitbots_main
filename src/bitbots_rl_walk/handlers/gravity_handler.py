import numpy as np
from transforms3d.euler import euler2mat
from transforms3d.quaternions import quat2mat

from bitbots_rl_walk.handlers.handler import Handler


class GravityHandler(Handler):
    def __init__(self, config_file: str):
        super().__init__(config_file=config_file)

        self._imu_data = None
        self._gravity = None

    # Callables
    def imu_callback(self, msg):
        self._imu_data = msg

    def get_data(self):
        gravity = (
            quat2mat(
                [
                    self._imu_data.orientation.w,
                    self._imu_data.orientation.x,
                    self._imu_data.orientation.y,
                    self._imu_data.orientation.z,
                ]
            )
            @ euler2mat(0, -0.0, 0)
        ).T @ np.array([0, 0, -1], dtype=np.float32)

        return gravity
