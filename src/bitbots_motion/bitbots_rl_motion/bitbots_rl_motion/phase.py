import numpy as np

# Please pay attention to the code in rl_node.py if you wanna change here sth.


class PhaseObject:
    _phase: np.ndarray = np.array([0.0, np.pi], dtype=np.float32)
    _phase_dt: float

    def __init__(self, config):
        super().__init__(config)

        self._control_dt = self._config["phase"]["control_dt"]
        self._gait_frequency = self._config["phase"]["gait_frequency"]
        self._phase_dt = 2 * np.pi * self._gait_frequency * self._control_dt

    def set_phase(self, new_phase):
        self._phase = new_phase

    def get_phase(self):
        return self._phase

    def get_phase_dt(self):
        return self._phase_dt
