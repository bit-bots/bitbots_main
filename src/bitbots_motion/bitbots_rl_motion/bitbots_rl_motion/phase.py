import numpy as np

# Please pay attention to the code in rl_node.py if you wanna change here sth.


class PhaseObject:
    _phase: np.ndarray = np.array([0.0, np.pi], dtype=np.float32)
    _phase_dt: float

    def __init__(self, node):
        self._node = node

        if self._node.get_parameter("phase.use_phase").value:
            self._control_dt = self._node.get_parameter("phase.control_dt").value
            self._gait_frequency = self._node.get_parameter("phase.gait_frequency").value
            self._phase_dt = 2 * np.pi * self._gait_frequency * self._control_dt
            self._use_phase = True
        else:
            self._control_dt = None
            self._gait_frequency = None
            self._phase_dt = None
            self._use_phase = False
            self._node.get_logger().warning("No phase was found! Using policy without phase!")

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

    def check_phase_set(self):
        return self._use_phase
