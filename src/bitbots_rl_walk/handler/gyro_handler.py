import numpy as np
from sensor_msgs.msg import Imu


# TODO: Adapt to software
class GyroHandler:
    def __init__(self, subscribers):
        self._gyro = {"subscriber": {"Imu": [Imu, "imu/data", self.imu_callback, 10]}, "name": "gyro"}

        # TODO:

    # Callables
    def imu_callback(self, msg):
        self._imu_data = msg

    def set(self, key, value):
        self._gyro[key] = value

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

    def get_confg(self):
        return self._gyro
