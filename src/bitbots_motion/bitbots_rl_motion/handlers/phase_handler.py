import numpy as np
from bitbots_rl_motion.handlers.handler import Handler


class PhaseHandler(Handler):
    _phase: np.ndarray = np.array([0.0, np.pi], dtype=np.float32)
    _phase_dt: float

    def __init__(self, config):
        super().__init__(config)

        self._control_dt = self._config["phase"]["control_dt"]
        self._gait_frequency = self._config["phase"]["gait_frequency"]
        self._phase_dt = 2 * np.pi * self._gait_frequency * self._control_dt

    def get_control_dt(self):
        return self._control_dt

    def get_gait_frequency(self):
        return self._gait_frequency

    def get_phase(self):
        return self._phase

    def get_phase_dt(self):
        return self._phase_dt
