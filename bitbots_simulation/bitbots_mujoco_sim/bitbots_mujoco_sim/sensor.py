import mujoco
import numpy as np


class Sensor:
    """Represents a single sensor, providing a clean interface to its value."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, name: str, ros_name: str):
        self._data = data
        self.ros_name: str = ros_name
        self.instance: int = model.sensor(name)
        self.id: int = self.instance.id

    @property
    def data(self) -> np.ndarray:
        """Gets the current sensor reading from sensordata array."""
        return self._data.sensordata[self.adr : self.adr + self.dim]

    @property
    def adr(self) -> int:
        """Gets the address of the sensor data in sensordata array."""
        return int(self.instance.adr)

    @property
    def dim(self) -> int:
        """Gets the dimension of the sensor data."""
        return int(self.instance.dim)
