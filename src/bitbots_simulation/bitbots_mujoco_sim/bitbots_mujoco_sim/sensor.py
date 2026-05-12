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
    def noisy_data(self) -> np.ndarray:
        """Gets the current sensor reading with measurement noise applied."""
        return self.data

    @property
    def adr(self) -> int:
        """Gets the address of the sensor data in sensordata array."""
        return int(self.instance.adr)

    @property
    def dim(self) -> int:
        """Gets the dimension of the sensor data."""
        return int(self.instance.dim)


class NoisySensor(Sensor):
    """Represents a sensor with per-start fixed bias and additive Gaussian noise."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        name: str,
        ros_name: str,
        gaussian_stddev: float = 0.0,
        bias_stddev: float = 0.0,
        rng: np.random.Generator | None = None,
        fixed_bias: np.ndarray | None = None,
    ):
        super().__init__(model, data, name, ros_name)
        self.gaussian_stddev = float(gaussian_stddev)
        self.bias_stddev = float(bias_stddev)
        self._rng = rng if rng is not None else np.random.default_rng()

        if fixed_bias is not None:
            self._fixed_bias = np.asarray(fixed_bias, dtype=float)
        elif self.bias_stddev > 0:
            self._fixed_bias = self._rng.normal(scale=self.bias_stddev, size=self.dim)
        else:
            self._fixed_bias = np.zeros(self.dim, dtype=float)

    @property
    def fixed_bias(self) -> np.ndarray:
        """Gets the fixed bias sampled at simulation start."""
        return self._fixed_bias

    @property
    def noisy_data(self) -> np.ndarray:
        """Gets the sensor reading with fixed bias and additive Gaussian noise."""
        return self.data + self._fixed_bias + self._rng.normal(scale=self.gaussian_stddev, size=self.dim)

    def reset_fixed_bias(self, fixed_bias: np.ndarray | None = None) -> None:
        """Resets the fixed bias for this sensor."""
        if fixed_bias is not None:
            self._fixed_bias = np.asarray(fixed_bias, dtype=float)
        elif self.bias_stddev > 0:
            self._fixed_bias = self._rng.normal(scale=self.bias_stddev, size=self.dim)
        else:
            self._fixed_bias = np.zeros(self.dim, dtype=float)


class QuaternionNoisySensor(NoisySensor):
    """Represents a quaternion sensor with noise and normalization."""

    @property
    def noisy_data(self) -> np.ndarray:
        noisy = super().noisy_data
        norm = np.linalg.norm(noisy)
        if norm == 0:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return noisy / norm


class FootPressureSensor(Sensor):
    """Represents a foot pressure sensor, providing access to individual pressure readings."""

    @property
    def force(self) -> float:
        """Gets the total force measured by the sensor."""
        return self.data[2]
