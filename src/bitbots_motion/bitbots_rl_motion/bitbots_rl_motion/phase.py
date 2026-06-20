import numpy as np
from rclpy.node import Node


class Phase:
    def __init__(self, node: Node):
        self._node = node
        self._use_phase = self._node.get_parameter("phase.use_phase").value
        self._phase = np.array(self._node.get_parameter("phase.initial_phase").value)
        self._control_dt = self._node.get_parameter("phase.control_dt").value
        self._gait_frequency = self._node.get_parameter("phase.gait_frequency").value
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

    def check_phase_set(self):
        return self._use_phase
