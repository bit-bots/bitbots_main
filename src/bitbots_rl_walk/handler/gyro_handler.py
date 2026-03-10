import numpy as np


class GyroHandler:
    def __init__(self):
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
