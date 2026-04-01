import numpy as np

# Please pay attention to the code in rl_node.py if you wanna change here sth.


class PhaseObject:
    _phase: np.ndarray = np.array([0.0, np.pi], dtype=np.float32)
    _phase_dt: float

    def __init__(self, config):
        self._control_dt = config["phase"]["control_dt"]
        self._gait_frequency = config["phase"]["gait_frequency"]
        self._phase_dt = 2 * np.pi * self._gait_frequency * self._control_dt
        self._obs_phase = None

    def set_phase(self, new_phase):
        self._phase = new_phase

    def set_obs_phase(self, new_obs_phase):
        self._obs_phase = new_obs_phase

    def get_phase(self):
        return self._phase

    def get_phase_dt(self):
        return self._phase_dt

    def get_obs_phase(self):
        return self._obs_phase
