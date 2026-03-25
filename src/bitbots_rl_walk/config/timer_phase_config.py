import numpy as np

CONTROL_DT = 0.02  # Control loop frequency in seconds
GAIT_FREQUENCY = 1.5  # Gait frequency in Hz


class TimerPhaseConfig:
    _phase: np.ndarray = np.array([0.0, np.pi], dtype=np.float32)
    _phase_dt: float

    def __init__(self):
        # Phase time
        self._control_dt = CONTROL_DT
        self._gait_frequency = GAIT_FREQUENCY
        self._phase_dt = 2 * np.pi * GAIT_FREQUENCY * CONTROL_DT

    def get_control_dt(self):
        return self._control_dt

    def get_gait_frequency(self):
        return self._gait_frequency

    def get_phase(self):
        return self._phase

    def get_phase_dt(self):
        return self._phase_dt
