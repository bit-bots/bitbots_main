import numpy as np
from bitbots_rl_motion.handlers.handler import Handler


class GyroHandler(Handler):
    def __init__(self, config):
        super().__init__(config)
        self._imu_data = None

    # Callables
    def imu_callback(self, msg):
        self._imu_data = msg

    def get_data(self):
        gyro = np.array(
            [
                self._imu_data.angular_velocity.x,
                self._imu_data.angular_velocity.y,
                self._imu_data.angular_velocity.z,
            ],
            dtype=np.float32,
        )

        return gyro
